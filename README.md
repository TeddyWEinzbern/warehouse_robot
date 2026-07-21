# Warehouse Robot Control System

Safety-oriented Arduino Uno firmware and a Python host console for a four-wheel
mecanum warehouse robot. Select the build profile explicitly when changing
hardware backends; `safe_idle` remains the no-actuator bring-up profile.

The UART drivetrain uses the motor board's own closed-loop speed controller:

- `$Car:` carries A/B/C/D target wheel speeds in m/s.
- `$Car_Pwm:` is a separate open-loop calibration mode.
- The Arduino does mecanum mixing, chassis-space ramps, safety interlocks,
  command scheduling, and encoder supervision. It does not contain another
  wheel-speed PID or PI loop.

The wheel order, signs, dimensions, counts/revolution, `$Car:` scaling, and
`Encoder_20ms` timing semantics are deliberately marked provisional. The
normal UART profile refuses to arm while
`ROBOT_DRIVE_CALIBRATION_QUALIFIED=0`.

## Runtime structure

```text
Python gamepad (10/20 Hz) ─┐
Local web dashboard ────────┼─> host safety runtime ── protocol v2 ──> Arduino mailbox
                            │                                      │
                            └─ browser may disconnect safely       ├─ 10 ms chassis ramp
                                                               ├─ 20 ms $Car: scheduler
Motor-board UART RX ── fixed parser/query arbiter <────────────┼─ 20 ms encoder query
                                                               ├─ 20 ms servo trajectory
Sonar echo edges ───────────────────────────────────────────────┴─ independent telemetry
```

Every firmware loop pass checks safety and pumps UART bytes. Periodic tasks use
rollover-safe accumulated deadlines, measured elapsed time, skip-not-catch-up
behavior, and missed-deadline counters. A scheduler overrun while armed is a
latched fault. Disarm, link timeout, E-stop, and critical faults bypass all
ramps and request the exact `$Car:0,0,0,0!` frame immediately.

More detail is in [protocol-v2.md](docs/protocol-v2.md) and
[raised-wheel-qualification.md](docs/raised-wheel-qualification.md).

## Build profiles

```sh
# No motor backend and no servo attachment
pio run -e safe_idle

# Raised-wheel UART qualification, capped at 200 mm/s
pio run -e uart_closed_loop_qualification

# Normal profile; builds but refuses ARM until qualification is promoted
pio run -e uart_closed_loop_robot

# Dedicated calibration profiles
pio run -e uart_open_loop_calibration
pio run -e arm_calibration
```

Arm servo calibration (interactive `warehouse-robot calibrate` session and the
BuildConfig write-back) is documented step by step in
[docs/arm-calibration.md](docs/arm-calibration.md).

The standard firmware profiles use 38400 baud, but the HC-05 data-mode baud must
match the configured module. The 9600-baud compatibility profiles reduce
firmware telemetry to 5 Hz and require the host's 9600-baud mode, which reduces
its control stream to 10 Hz. The Arduino's 20 ms motor schedule is unchanged:

```sh
pio run -e uart_closed_loop_qualification_9600
pio run -e uart_closed_loop_robot_9600
```

## Python telemetry and tuning console

Create an environment and install the pinned runtime:

```sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e '.[test]'
warehouse-robot list-ports
```

Run the safety runtime and loopback-only dashboard:

```sh
# Use the exact /dev/cu.HC-05... path reported by `warehouse-robot list-ports`.
# This example requires a matching 9600-baud firmware profile.
warehouse-robot run --port /dev/cu.HC-05 --baud 9600
# Open http://127.0.0.1:8765
```

An optional standalone console bundle can be produced after installing the
packaging extra:

```sh
python3 -m pip install -e '.[package]'
python3 -m PyInstaller warehouse_robot_gui.spec
```

Use `--no-gamepad` for telemetry and disarmed calibration only. The browser is
not in the control or safety path: closing it does not stop the Python runtime,
the configured 10/20 Hz control stream, serial supervision, or E-stop handling.
WebSocket clients receive coalesced immutable snapshots through one-element
queues.

The dashboard shows:

- requested and ramped chassis velocity;
- `$Car:` A/B/C/D targets, measured speed, target-minus-measured error;
- encoder validity, age, query state, sample interval, and timing semantics;
- explicit PWM unavailability in `$Car:` mode, or signed PWM in open-loop modes;
- arm servo targets, sonar pairs, battery, state, E-stop, faults, and warnings;
- loop gap, missed deadlines, query timeouts, RX overflows, dt clamps, and
  coalesced telemetry;
- session-only servo, PWM, speed, acceleration, reversal, encoder mapping,
  arm geometry/workspace, sensor-offset, assist/cargo, and response-profile
  tuning, plus host-only gamepad deadzone and response-power tuning.

Runtime parameter commits are revisioned, atomic, validated against compiled
hard ceilings, and accepted only in `DISARMED`. They reset on firmware restart.
Validated values can then be reviewed and copied into source defaults.
Arm geometry/workspace commits are intentionally available only in the
`arm_calibration` profile so their parsing and validation code is absent from
the flash-constrained normal drivetrain image.

The explicit arm calibration command is typed protocol v2, not a production
ASCII bypass:

```sh
warehouse-robot calibrate-joint \
  --port /dev/cu.usbmodemXXXX --joint 0 --angle 90
```

## UART robot wiring

| Function | Uno pin |
| --- | --- |
| Motor-board TX -> Uno RX | D0 |
| Motor-board RX <- Uno TX | D1 |
| HC-05 TX -> Uno RX | A5 |
| HC-05 RX <- Uno TX | A4 through a 3.3 V-safe divider |
| Base / shoulder / elbow / gripper servo | D3 / D5 / D6 / D9 |
| Sonar trigger group 0 / group 1 | D2 / D13 |
| Group 0 front / left / right echo | D4 / D8 / D11 |
| Group 1 front / left / right echo | D7 / D10 / D12 |

All modules require a common ground. Motors and servos require suitable
external power. A physical motor-power cutoff remains mandatory for hardware
qualification. Disconnect the D0/D1 motor-board link during Uno upload.

The D13 second-trigger wiring and HC-05 baud selection remain hardware approval
gates. Sonar and arm motion are disabled by the qualification profile.

## Verification

```sh
PYTHONPATH=scripts python3 -m unittest discover -s tests -v
python3 -m compileall -q scripts tests
pio test -e native
pio run -e safe_idle \
  -e uart_closed_loop_qualification -e uart_closed_loop_robot \
  -e uart_closed_loop_qualification_9600 -e uart_closed_loop_robot_9600 \
  -e uart_open_loop_calibration -e arm_calibration
```

These checks verify software behavior and Uno resource limits. They do not
replace the raised-wheel, E-stop, wiring, encoder, or 30-minute jitter tests.

## Legacy/debug isolation

Production communication accepts protocol v2 only. Direct-wheel `W:`, motor
`M:`, chassis `C:`, and joint `J:` ASCII paths are not referenced by the
runtime. The old compatibility scripts and `CalibrationConsole` remain in the
tree only for post-HIL removal review; `CalibrationConsole.cpp` is explicitly
excluded from every firmware profile and actuator-capable profiles do not call
it.
