# Protocol v2 and motor-board contract

## Host envelope

All host/Arduino messages use:

```text
version:u8 = 2
message_type:u8
sequence:u8
payload_length:u8 (0..27)
payload
crc8(poly 0x07, init 0)
COBS encoding
0x00 delimiter
```

Malformed, oversized, or CRC-invalid messages are dropped. Control axes are
signed `int16` values in `[-1000, 1000]`. The E-stop is a level bit in the
Control payload; arm, disarm, clear-E-stop, and clear-fault are separate typed
requests. A clear request never implies arm.

| Type | Value | Payload |
| --- | ---: | --- |
| Hello | `0x01` | empty |
| Control | `0x02` | six `i16` axes, `i8` gripper, `u16` buttons, `u8` flags |
| Arm / Disarm | `0x03` / `0x04` | empty |
| Clear E-stop / fault | `0x05` / `0x06` | empty |
| Parameter set | `0x10` | group, index, expected revision, group data |
| Calibration command | `0x11` | joint, raw degrees |
| Parameter snapshot request | `0x12` | empty |
| Drive calibration command | `0x13` | mode (`0` open-loop percent, `1` closed-loop mm/s), board channel, `i16` value, `u16` duration ms |
| Hello / status telemetry | `0x80` / `0x81` | capability and safety state |
| Drive command / feedback | `0x82` / `0x83` | physical target and measured speed domains |
| Encoder totals / scheduler | `0x84` / `0x85` | independent freshness and deadline health |
| Sensor/arm / open-loop PWM | `0x86` / `0x87` | calibrated readings and mode-specific PWM |
| Parameter snapshot | `0x90` | group, index, revision, group data |
| ACK / NACK | `0x91` / `0x92` | commit result |

The golden host Control frame is tested in `tests/test_protocol.py` and the
firmware parser uses the same version, CRC, COBS, and exact 16-byte payload.

Drive calibration commands are accepted only by the `calibration` image while
`DISARMED`: values are capped (±100 %, ±200 mm/s, ≤10 s), the frame repeats on
the 50 ms keepalive slot, and expiry falls back to the `$Car:0,0,0,0!` zero
frame. Profile enum slots 0–5 are retired (safe idle, L293D, qualification,
open-loop calibration, arm-only calibration); the live profiles are
`UartClosedLoopRobot` (3), `UartOpenLoopRobot` (6), and `Calibration` (7).

## Telemetry meanings

- `controller_targets` are the four values sent to motor-board channels A/B/C/D
  after command order and sign mapping.
- `zero_crossing_mask` identifies longitudinal, lateral, and yaw axes currently
  held at zero before a commanded reversal.
- `measured_speeds` are logical wheel speeds converted from encoder feedback.
- `speed_errors` are logical target minus measured wheel speed.
- The host derives each logical target as `measured_speeds + speed_errors`; the
  dashboard does not align board channel A/B/C/D values with logical rows unless
  the current command map says they are the same.
- `feedback_age_ms` is based on arrival of the most recent accepted increment
  reply, regardless of conversion semantics.
- `raw_increments` and encoder totals have independent validity and age.
- `$Car:` mode sets PWM validity false. A speed target is never relabeled PWM.
- `$Car_Pwm:` reports signed percent times 100, carried in the separate
  open-loop PWM message. (The retired L293D backend used signed raw 8-bit PWM;
  its `pwm_unit` and profile enum slots remain reserved.)

## Runtime parameter groups

Parameter commits are session-only and atomic. The expected revision must
match, the robot must be `DISARMED`, and the complete candidate configuration
must pass validation.

1. Servo lower/upper limits, center offsets, and directions.
2. Reserved (was per-motor open-loop PWM for the retired L293D backend).
3. Chassis speed and final wheel ceilings.
4. Direction-specific acceleration/deceleration and zero-crossing settings.
5. Encoder geometry, feedback channel/sign map, command channel/sign map, and
   sample semantics.
6. Per-sonar offsets.
7. Operator/cargo/assist ceilings.
8. Low/normal/aggressive response selection.
9. Per-channel `$Car_Pwm:` minimum/maximum percentages and signs.
10. Arm link/workspace geometry and clearance/preset/stow positions. This group
    is accepted and snapshotted only by the dedicated `calibration` image;
    validated values are promoted to compiled defaults before enabling the arm.

Command timeout, encoder stale timeout, scheduler fault thresholds, and
compiled hard ceilings are intentionally not ordinary GUI tunables.

## Authoritative UART reference

The ignored local vendor examples were read as the authority; they were not
copied or committed. Relevant snapshot hashes:

| Local reference | SHA-256 | What it establishes |
| --- | --- | --- |
| `examples/UART历程/驱动电机/uart_test/uart_test.ino` | `290154331d2b73dbf90639d3d9f0672b0165e2118464a8ffb5ccd1c7700a80c5` | init spelling, `$Car:`, `$Car_Pwm:`, A/B/C/D examples |
| `examples/UART历程/回读数据/uart_test/uart_test.ino` | `e9b35df163a3640f1e9c25a7fc7af5e3d292026ec6a87bed4793b23dfb09aa14` | query and reply spelling/field widths |
| `examples/C25 v5/User/app_motor.c` | `6b2ab0ed6731dacab09db950f3e6c889f490302317d98456100d5681b0efb406` | `$Car:` values are m/s wheel targets; controller owns PID/PWM |
| `examples/C25 v5/User/app_motor.h` | `98b8ebd4293a4ebc8f8f5128ee28b5cd8b356f58899a3b47946d1f5d65dad162` | 50 Hz PID, 60 mm wheel, 4680 counts, 160/170 mm geometry |
| `examples/C25 v5/User/Components/y_global/y_global.c` | `fa295dcbea4daa022c97cbddea3559eea51c35fba4345dacf03d3f09ff155083` | `$Car:` parser and `cmdOk` noise |

The Arduino implementation intentionally replaces the examples' `String`,
`readStringUntil`, delay, and response waits with bounded state machines. That
changes scheduling mechanics, not the authoritative command syntax.

The reference controller source confirms that its internal motor task samples
encoder counters and runs four PID controllers at 50 Hz. Even so, wheel order,
signs, scale, and whether the exposed `Encoder_20ms` reply is a fixed internal
sample remain qualification results rather than accepted defaults.
