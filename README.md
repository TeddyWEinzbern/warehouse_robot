# Warehouse Robot Control System

Safety-oriented Arduino Uno firmware and a Python host console for a four-wheel
mecanum warehouse robot. The normal deployable firmware environments are
`robot` and `calibration`; each has an explicitly degraded `_unsafe`
counterpart for raised-wheel fault diagnosis only. Host-native tests use
`native`, plus `native_calibration` and `native_unsafe`.

The UART motor board owns the wheel-speed controller:

- `$Car:` carries four closed-loop wheel-speed targets in m/s.
- `$Car_Pwm:` carries four open-loop PWM-percent targets when the open-loop
  build flag is selected.
- Startup selects vendor motor type `1`, matching the installed TT motors.
- The Uno owns mecanum mixing, chassis ramps, safety interlocks, scheduling,
  arm control, optional sonar, and supervision. It does not add another
  wheel-speed PID loop.
- The Uno updates the chassis ramp and attempts a motor target every 10 ms;
  encoder-increment queries remain every 20 ms. The host control stream remains
  30 Hz and the vendor board's own wheel-speed PID remains 50 Hz, so the 10 ms
  target period is not an end-to-end 10 ms response guarantee.

Compiled chassis limits are 450 mm/s longitudinal, 250 mm/s lateral, and
1,500 mrad/s yaw. Ramp defaults are longitudinal 600/1,500 mm/s²
acceleration/deceleration, lateral 500/1,500 mm/s², yaw 3,000/6,000 mrad/s²,
with a 40 ms hold at a commanded direction reversal.

Wheel order, signs, geometry, counts/revolution, arm limits, and qualification
are established with [the calibration checklist](docs/calibration.md). A
calibration flag is evidence that the corresponding procedure was completed;
it is not a feature-enable flag.

## Runtime structure

```text
Gamepad at 30 Hz ───────────┐
                            ├─ Python safety runtime ─ protocol v3 ─ HC-06 ─ Uno
Two-button local WebUI ─────┘          │                 A4/A5       │
                                      │                              ├─ safety gates
                                      └─ status/events                ├─ arm scheduler
                                                                     ├─ chassis scheduler
Motor-board UART D0/D1 <──────────────── bounded parser/query arbiter └─ optional sonar
```

The normal host stream is deliberately small:

- compact control at 30 Hz at both supported baud rates;
- dedicated short E-stop frames, sent redundantly until critical status
  confirms the latch;
- compact critical status at 2 Hz and immediately after safety-state changes;
- no periodic detailed telemetry and no remote runtime-parameter writes.

Calibration uses explicit on-demand command/reply messages. See
[protocol-v3.md](docs/protocol-v3.md) for the wire contract.

Every firmware loop pass applies link timeout, E-stop, fault, and enable gates
before motion. In the normal images, disarm, link loss, E-stop, and critical
faults request the exact motor-board zero command without waiting for a ramp.

## Build environments

```sh
# Combined arm, drive, and sensor calibration surface.
# The operations available still depend on the feature flags.
pio run -e calibration

# Normal robot image. Closed-loop control and 9600 baud are the defaults.
pio run -e robot

# Raised-wheel diagnosis only: fault bits remain visible but cannot select
# the firmware FAULT state.
pio run -e robot_unsafe
pio run -e calibration_unsafe

# Host-native domain and protocol tests.
pio test -e native
pio test -e native_calibration
pio test -e native_unsafe
```

The feature flags are intentionally separated by meaning:

| Flag | Meaning |
| --- | --- |
| `ROBOT_DRIVER_ENABLED` | Structural backend choice. Shipped Uno builds use `1` for the UART motor-driver class; native tests use `0`/Null. |
| `ROBOT_DRIVER_CONTROL_CLOSE` | Select `$Car:` closed-loop board commands. Exactly one driver-control flag must be enabled. |
| `ROBOT_DRIVER_CONTROL_OPEN` | Select `$Car_Pwm:` open-loop board commands. |
| `ROBOT_DRIVE_ENABLED` | Functional safety gate. `0` prevents motor-board initialization, polling, and commands. The real UART object remains compiled and allocated. |
| `ROBOT_DRIVE_CALIBRATED` | Records completed drive calibration. It does not enable the drive. |
| `ROBOT_ARM_ENABLED` | Functional arm-motion gate. The arm implementation and object remain present when disabled. |
| `ROBOT_ARM_CALIBRATED` | Records completed arm calibration. It does not enable the arm. |
| `ROBOT_SENSOR_ENABLED` | Functional sonar gate; omitted means `0`. Disabled sensors remain allocated but their pins and scheduler are not activated. The shipped `calibration` environment selects `1` so sensors can be qualified. |
| `ROBOT_CALIBRATION` | Selects the calibration firmware personality. Set by the `calibration` environment. |
| `ROBOT_FAULT_STATE_UNSAFE` | Makes every fault bit telemetry-only for state selection: bits still latch and remain visible, but cannot move an `_unsafe` image into `FAULT`. Startup qualification, E-stop, link-loss DISARM, and host-side fault-free ARM remain enforced. |
| `ROBOT_HOST_BAUD` | HC-06 UART baud: `9600UL` by default or `38400UL`. |

The former `ROBOT_DRIVER_TIMEOUT_UNSAFE` spelling remains a compatibility
alias, but now selects the same all-fault unsafe behavior.

Preprocessor decisions are kept at structural boundaries: defaults and
validation live in `BuildConfig.h`, the backend type is selected once in
`RobotApplication`, and calibration-only wire/report code is fenced at the
application/driver boundary. Ordinary feature enablement uses the fixed
objects plus `constexpr` safety gates instead of scattering per-method
`#ifdef` branches or changing packet layouts.

`ROBOT_DRIVE_ENABLED=0` is the supported arm-only robot test configuration.
Keep motor power physically isolated as well; a compile-time flag is not an
emergency disconnect. The calibration image observes the same enable flags, so
disabled hardware does not become active merely because calibration firmware
is running.

`robot_unsafe` and `calibration_unsafe` never select `FAULT` because of a fault
bit. The bits still latch in critical status, and the images always publish
`fault_state_unsafe`; encoder timeout, sign, and mismatch also retain their
specific `*_ignored` warnings. An already-armed unsafe robot can therefore
remain globally ARMED after malformed/implausible encoder data, stall,
scheduler overrun, or arm faults; subsystem-local fallbacks such as holding the
last arm command still apply. The host still requires fault-free status for a
new ARM; while DISARMED it exposes CLEAR FAULT for an explicit retry. These
images must only be used with every wheel raised and an immediate physical
power cutoff available. Never deploy either image for ground operation.

To select open-loop board control, replace
`-DROBOT_DRIVER_CONTROL_CLOSE=1` with
`-DROBOT_DRIVER_CONTROL_OPEN=1` in the required environment. Exactly one must
be `1`; an omitted control flag defaults to `0`. To enable normal-image sonar,
add `-DROBOT_SENSOR_ENABLED=1` only after the wiring and live readings have
been qualified.

## HC-06 host link

The supported Bluetooth target is an HC-06 Bluetooth Classic serial module.
The computer opens its RFCOMM serial port and the module transparently bridges
bytes to the Uno's A4/A5 NeoSWSerial link. Robot firmware does not configure
the module with AT commands.

The project default is **9600 baud**, matching the manufacturer's HC-06
default. `BuildConfig.h` supplies that fallback when no build override is
present. The HC-06, Uno firmware, and Python host must use the same baud:

```ini
; no ROBOT_HOST_BAUD build flag -> 9600
```

```sh
warehouse-robot run --port /dev/cu.HC-06 --baud 9600
```

38400 remains selectable:

```ini
-DROBOT_HOST_BAUD=38400UL
```

```sh
warehouse-robot run --port /dev/cu.HC-06 --baud 38400
```

Changing the build flag or `--baud` does not reconfigure the HC-06. Follow the
[step-by-step HC-06 38400 guide](docs/hc-06-38400.md) before selecting 38400.
It includes isolated wiring, firmware identification, exact commands,
verification, rollback, and clone/command-set failure cases.

The link is bidirectional and uses receive/transmit windows; do not add
unbounded debug text to A4/A5. A Bluetooth settings page may show the module as
disconnected while no process owns the RFCOMM port. Qualify the connection by
opening the enumerated serial device and receiving a valid protocol-v3
HELLO/status response.

## Python runtime and WebUI

Create an environment and install the pinned runtime:

```sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e '.[test]'
warehouse-robot list-ports
```

Start the runtime:

```sh
warehouse-robot run --port /dev/cu.HC-06 --baud 9600
# Open http://127.0.0.1:8765
```

Useful options include `--web-port`, `--no-gamepad`, and
`--reconnect-attempts`. Use `--no-gamepad` for connection and safety-status
diagnostics, not robot motion.

The loopback-only WebUI is a safety console, not a tuning dashboard. It shows:

- HC-06 target, configured serial device, and baud;
- connection state, protocol verification, status age, and last accepted
  control sequence;
- robot state plus fault and warning badges;
- firmware profile, driver mode, and enabled/calibrated subsystem summary;
- controller state, pending safety action, and target/actual 30 Hz timing
  including interval and missed-control counts;
- timestamped local/fatal errors, firmware fault-bit transitions, and recent
  host safety events.

It has exactly two contextual safety controls:

1. `ARM`, `DISARM`, or `CLEAR FAULT`, according to the current state;
2. `E-STOP` or `CLEAR E-STOP`, according to the current state.

On SDL-recognized Xbox-style controllers, including the Xbox One Elite Series
2, the named `Menu`/Start control requests ARM and `View`/Back requests CLEAR
E-STOP; raw platform-specific button numbers are not used. Menu is
edge-triggered and does not toggle or DISARM. The ARM request still requires
fresh fault-free DISARMED status, completed calibration, a cleared E-stop, and
0.6 seconds of neutral control. DISARM and E-STOP remain available from the
loopback WebUI, and controller loss still latches E-stop.

There are no wheel, encoder, sonar, arm, scheduler, or tuning panels and no
remote parameter API. Closing the browser does not stop the Python safety
runtime, control heartbeat, serial supervision, or E-stop handling.

An optional standalone console bundle can be built with:

```sh
python3 -m pip install -e '.[package]'
python3 -m PyInstaller warehouse_robot_gui.spec
```

## Calibration

The interactive protocol-v3 calibration session runs over the same A4/A5
HC-06 link:

```sh
warehouse-robot calibrate --port /dev/cu.HC-06 --baud 9600
```

Arm jogs, drive spins, encoder reads, sensor reads, and the qualification-only
stack-watermark read are explicit on-demand requests. The calibration tool can
export reviewed values for the compiled defaults; the normal robot image does
not accept remote tuning writes or expose the stack instrument.

Follow [docs/calibration.md](docs/calibration.md) from power isolation through
source write-back and raised-wheel qualification. Sonar wiring and the
default-off sensor gate are documented in [docs/sensors.md](docs/sensors.md).

## UART robot wiring

| Function | Uno pin |
| --- | --- |
| Motor-board TX -> Uno RX | D0 |
| Motor-board RX <- Uno TX | D1 |
| HC-06 TX -> Uno RX | A5 |
| HC-06 RX <- Uno TX | A4 through a 3.3 V-safe divider/level shifter |
| Base / shoulder / elbow / gripper servo | D3 / D5 / D6 / D9 |
| Sonar trigger group 0 / group 1 | D2 / D13 |
| Group 0 front / left / right echo | D4 / D8 / D11 |
| Group 1 front / left / right echo | D7 / D10 / D12 |

All modules require a common ground. Motors and servos require suitable
external supplies. A physical motor-power cutoff remains mandatory for
raised-wheel work. Disconnect the D0/D1 motor-board link during Uno upload.

The HC-06 RX / Uno TX path must be 3.3 V safe because the Uno transmits 5 V
logic. Power requirements depend on whether the installed part is a bare 3.3 V
module or a carrier with its own regulator; follow its marking and schematic
rather than the product name.

Sonar is disabled by default. D13 wiring and live-sensor qualification remain
hardware gates before setting `ROBOT_SENSOR_ENABLED=1`.

## Verification

```sh
PYTHONPATH=scripts python3 -m unittest discover -s tests -v
python3 -m compileall -q scripts tests
pio test -e native
pio test -e native_calibration
pio test -e native_unsafe
pio run -e robot -e calibration
pio run -c "$PWD/.github/platformio-variants.ini" \
  -e ci_robot_qualified -e ci_robot_38400 -e ci_robot_open \
  -e ci_robot_arm_only -e ci_robot_sensor -e ci_calibration_38400
```

The absolute alternate-config path is intentional: PlatformIO must propagate
that file into its nested build process. These commands verify software
behavior, build the two supported Uno images, and compile the CI-only flag
matrix. They do not replace:

- raised-wheel drive qualification;
- physical E-stop and motor-power-cutoff tests;
- arm alignment and collision-clearance checks;
- HC-06 bidirectional link tests at the selected baud;
- sonar wiring/live-reading checks when enabled;
- a long-duration scheduler and stack-watermark run.

Production host communication accepts protocol v3 only. Legacy ASCII
direct-wheel, chassis, motor, or joint commands are not part of the runtime
contract.
