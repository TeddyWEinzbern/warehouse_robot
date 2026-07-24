# Protocol v3 and motor-board contract

Protocol v3 is intentionally split into a compact real-time lane and a typed
command/reply lane. Production traffic contains 30 Hz control, a dedicated
E-stop packet, HELLO, and compact critical status. Detailed reports exist only
as on-demand calibration replies.

All multibyte integers in the generic lane are little-endian.

## Framing shared by both lanes

Every raw packet ends with:

```text
crc8(poly 0x07, init 0)
```

The CRC covers every preceding raw byte. The raw packet is COBS encoded and a
single `0x00` delimiter is appended. A malformed COBS packet, unexpected
length, reserved bit, out-of-range value, unknown required type, or CRC failure
is rejected as a whole.

The receive parser is bounded: it never waits for text terminators, allocates a
dynamic string, or accepts a valid prefix with trailing bytes.

## Compact control lane

### Control packet

The raw control packet is nine bytes:

```text
byte 0    01ssssss    class 01 plus 6-bit sequence
byte 1    forward    signed i8 percentage, -100..100
byte 2    turn       signed i8 percentage, -100..100
byte 3    strafe     signed i8 percentage, -100..100
byte 4    arm yaw    signed i8 percentage, -100..100
byte 5    arm reach  signed i8 percentage, -100..100
byte 6    bits 0..1 arm height; bits 2..3 gripper; bits 4..7 zero
byte 7    action bits; bits 6..7 zero
byte 8    CRC-8
```

COBS plus the delimiter makes the normal wire frame 11 bytes. The sequence
wraps modulo 64. The complete packet must validate before it replaces the
current mailbox; partial or invalid packets cannot update only some axes.

Each signed percentage is expanded to the host's `-1000..1000` permille
domain by multiplying by 10. Quantization therefore has a 10-permille step.

Each two-bit field in byte 6 uses:

| Code | Arm height | Gripper |
| ---: | --- | --- |
| `0` | neutral | neutral |
| `1` | down | open |
| `2` | up | close |
| `3` | invalid; reject the complete packet | invalid; reject the complete packet |

Byte 7 assigns bit 0 `PresetLeft`, bit 1 `PresetFront`, bit 2 `PresetRight`,
bit 3 `PresetStow`, bit 4 `StartAssist`, and bit 5 `CancelAssist`. These are
ordinary operator intents. None clears a fault, clears E-stop, or implies ARM.

The host sends control at 30 Hz at both 9600 and 38400 baud. Firmware link
freshness is based on accepted control packets, not raw serial activity.

### Dedicated E-stop packet

The raw E-stop packet is two bytes:

```text
byte 0    10ssssss    class 10 plus 6-bit sequence
byte 1    CRC-8
```

COBS plus the delimiter makes a four-byte wire frame. One valid packet latches
E-stop. The normal host runtime sends three frames for each urgent attempt,
then retries every 250 ms until a fresh critical-status message confirms
`ESTOP`. The calibration tool, which has no periodic status stream, uses a
three-frame fail-closed burst at startup and shutdown. Neither inserts an
E-stop level into every control packet. Clearing E-stop is a separate typed
command and never arms the robot.

Class prefixes `00` and `11` are not additional compact-frame classes. A raw
packet beginning with the literal protocol version `0x03` is instead parsed
as the generic lane below.

## Generic typed lane

Generic raw packets use:

```text
version:u8 = 3
message_type:u8
sequence:u8
payload_length:u8
payload[payload_length]
crc8
```

| Direction | Type | Value | Purpose |
| --- | --- | ---: | --- |
| Host → Uno | Hello | `0x01` | Start/verify a protocol-v3 session |
| Host → Uno | Arm | `0x03` | Request ARM through the complete safety gate |
| Host → Uno | Disarm | `0x04` | Request immediate safe disarm |
| Host → Uno | Clear E-stop | `0x05` | Clear a latched E-stop when neutral and otherwise eligible |
| Host → Uno | Clear fault | `0x06` | Clear eligible latched faults; does not ARM |
| Host → Uno | Calibration arm move | `0x10` | Calibration-only guarded arm movement |
| Host → Uno | Calibration joint reference | `0x11` | Calibration-only joint reference/update |
| Host → Uno | Calibration drive spin | `0x12` | Calibration-only bounded wheel command |
| Host → Uno | Calibration drive read | `0x13` | Request one drive/encoder report |
| Host → Uno | Calibration sensor read | `0x14` | Request one sonar report |
| Host → Uno | Calibration arm read | `0x15` | Request one protected arm-state report |
| Host → Uno | Calibration system read | `0x16` | Request the qualification-only stack-watermark report |
| Uno → Host | Hello response | `0x80` | Protocol/build configuration |
| Uno → Host | Critical status | `0x81` | Compact state, fault/warning, sequence, and link-alive status |
| Uno → Host | Calibration report | `0x90` | Reply to an explicit calibration read/action |
| Uno → Host | ACK | `0x91` | Typed command accepted |
| Uno → Host | NACK | `0x92` | Typed command rejected with a reason |

Sequence values in this lane are full `u8` values. Calibration reports and
ACK/NACK replies echo the request sequence so the host can match the
outstanding operation. HELLO responses and critical status use the Uno's
independent transmit sequence.

ARM, DISARM, clear-E-stop, and clear-fault are distinct safety requests. They
do not generate ACK/NACK traffic; the host confirms their effect only from a
fresh critical-status message. ACK/NACK is reserved for calibration
transactions, and no accepted command makes another safety transition
implicit.

### Generic payload layouts

All reserved values and unexpected payload lengths are rejected. Field order
below is wire order; `i16`, `u16`, and `i32` fields are little-endian.

| Message | Payload |
| --- | --- |
| Hello request | empty |
| Arm / Disarm / Clear E-stop / Clear fault | empty |
| Calibration arm move | `joint:u8, degrees:u8` |
| Calibration joint reference | `joint:u8, lower:u8, upper:u8, center_offset:i8, direction:i8` |
| Calibration drive spin | `mode:u8, channel:u8, value:i16, duration_ms:u16` |
| Calibration drive read | `page:u8`; `0` counts, `1` speeds |
| Calibration sensor read / arm read / system read | empty |
| Hello response | `profile:u8, arm_enabled:u8, drive_enabled:u8, sensor_enabled:u8, driver_mode:u8, arm_calibrated:u8, drive_calibrated:u8, baud_div_1200:u8` |
| Critical status | `state:u8, faults:u16, warnings:u16, last_control_sequence:u8, link_alive:u8` |
| ACK | `accepted_type:u8` |
| NACK | `rejected_type:u8, reason:u8` |

The shipped `profile` values are `3` closed-loop `robot`, `6` open-loop
`robot`, and `7` `calibration`. Enabled/calibrated/link fields are encoded as
`0` or `1`. `driver_mode` is `0` for no concrete driver, `1` for open-loop,
and `2` for closed-loop. `baud_div_1200` is `8` at 9600 baud and `32` at
38400 baud.

Critical `state` values are `0` Boot, `1` Disarmed, `2` Armed, `3` E-stop, and
`4` Fault. Fault bits are:

| Bit | Fault |
| ---: | --- |
| 0 | scheduler overrun |
| 1 | drive initialization |
| 2 | encoder stale |
| 3 | encoder malformed |
| 4 | encoder implausible |
| 5 | encoder sign |
| 6 | drive stall |
| 7 | drive mismatch |
| 8 | arm target |

Warning bits are bit 0 drive unqualified and bit 1 arm target limited.
NACK reasons are `1` malformed, `2` unsupported, `3` invalid state, and `4`
validation failure.

A calibration report starts with `kind:u8`:

| Kind | Remaining payload |
| ---: | --- |
| `1` arm | four last-commanded raw servo targets, one `u8` per joint; these are not position feedback |
| `2` drive counts | four `i16` increments, four `i32` totals, `valid_mask:u8` |
| `3` drive speed | four `i16` millimetres-per-second values, `valid_mask:u8` |
| `4` sensor | six `u16` millimetre distances, `valid_mask:u8` |
| `5` system | `minimum_untouched_stack_bytes:u16` |

Calibration drive-spin mode `0` is open-loop percent and mode `1` is
closed-loop millimetres per second. Channels are `0..3`; the compiled limits
are ±100 percent, ±200 mm/s, and 10 seconds. A zero-duration, zero-value spin
is the explicit stop request.

## HELLO and critical status

HELLO establishes all of the following before the host treats a port as the
robot link:

- protocol version 3;
- the selected `robot` or `calibration` personality;
- selected driver control mode;
- host-link baud;
- enabled and calibrated state for drive and arm, plus whether sonar is
  enabled.

The first valid critical-status message supplies the current critical safety
state. A HELLO response does not duplicate that recurring state payload.

Disabled subsystems remain compiled and allocated, so there is no
variable-length capability bitmap. HELLO contains fixed enabled/calibrated
fields that describe whether each subsystem may operate; those fields are not
repeated in every critical-status packet.

In the normal `robot` image, critical status is transmitted:

- periodically at 2 Hz at both supported baud rates; and
- promptly after state, fault, warning, or link-alive changes.

It is the only production periodic telemetry. Its recurring payload is limited to robot
state, faults, warnings, the last accepted control sequence, and link-alive
state. Together with the prior HELLO response, that is enough to render
connection verification, status age, critical safety state, and the fixed
enabled/calibrated summary. It does not contain wheel arrays, encoder traces,
sonar distances, servo positions, scheduler counters, or mutable tuning
parameters.

The host treats status age and the control-link watchdog independently. A live
browser is not proof of a live Arduino link, and arbitrary incoming bytes are
not proof of a valid protocol session.

The `calibration` image has no unsolicited periodic status. It is a strict
one-request/one-reply transport: after HELLO, the host must wait for each
calibration ACK/NACK/report before sending the next generic request. Its
startup neutral stream and the explicit `stress` qualification command are
the only compact-control bursts. Those streams anchor each 33.3 ms deadline
after the preceding host write completes, so a blocked RFCOMM writer lowers
the measured rate instead of triggering a catch-up burst or consuming the
Uno's reverse-direction window.

## Calibration-only reports

Calibration commands are accepted only by the `calibration` image, while
DISARMED, and only for the corresponding enabled subsystem. Every actuator
request remains bounded by compiled limits.

- Arm movement and joint-reference commands use the protected arm path.
- The first command for every servo must be raw 90 degrees; both host and
  firmware reject an arbitrary first attachment target.
- Drive spins are channel-, magnitude-, and duration-limited and return to the
  exact zero command on completion, cancellation, or timeout.
- E-stop or DISARM cancels pending calibration servo motion and forces the
  exact motor zero path.
- Drive/encoder data is returned only after an explicit drive-read request.
- Sonar data is returned only after an explicit sensor-read request.
- Arm state is returned only after an explicit arm-read request.
- Minimum untouched stack is returned only after an explicit system-read
  request.
- Exported values are reviewed and written into source defaults before the
  matching `*_CALIBRATED` flag is set.

The normal `robot` image does not accept parameter writes, parameter snapshots,
or detailed-report requests. Protocol v3 has no remote tuning transaction or
revisioned runtime-configuration API.

### Calibration stack-watermark instrument

At the end of `RobotApplication::begin()`, after device initialization and
scheduler setup, calibration firmware briefly masks interrupts and paints
`0xA5` from the current heap end through 48 bytes below the entry stack
pointer. A kind-5 report scans the consecutive untouched bytes from the heap
end. Its result is a conservative lifetime minimum since that paint:

- the deliberate 48-byte unpainted gap is counted as used, not free;
- constructor and `begin()` stack use before painting is also not claimed;
- a reset or power cycle repaints the region for a new qualification run;
- repeated reads without reset preserve the lowest observed headroom.

The calibration REPL commands `stack` and `system` request this report and mark
`>=256` bytes PASS. Its `stress` command supplies neutral compact control at
a 30 Hz target while already-started motor, sonar, and servo work continues,
and reports its actual rate, maximum interval, and missed slots. Require zero
missed slots and investigate any material rate shortfall. Exercise that
combined load, then request each largest calibration report before the final
read. A lighter manual command sequence is useful for diagnosis but does not
replace the combined-load qualification. The read itself briefly masks
interrupts so an ISR cannot deepen the stack behind the scan cursor.

The watermark is a calibration-only release instrument, not production
telemetry or a debugging console. It measures SRAM headroom; it does not prove
NeoSWSerial/motor-UART coexistence or servo timing, which still require the
logic-analyzer run below.

## Scheduling and HC-06 coexistence

With 8N1 framing, the wire can carry `baud / 10` bytes per second. Ignoring all
other traffic, an 11-byte control frame therefore has a mathematical ceiling
of 87.3 Hz at 9600 and 349.1 Hz at 38400. Those values are not safe operating
targets: they leave no allowance for reverse traffic, scheduling jitter,
motor-board UART service, or servo interrupts.

The selected 30 Hz stream consumes 330 bytes/s. A 14-byte critical-status frame
at 2 Hz adds 28 bytes/s, for 358 bytes/s of normal steady-state wire traffic:

| Host baud | Control airtime | Steady wire use | Nominal wire reserve |
| ---: | ---: | ---: | ---: |
| 9600 | 11.46 ms/frame | 37.3% | 62.7% |
| 38400 | 2.87 ms/frame | 9.3% | 90.7% |

HELLO, typed safety commands, and E-stop retries are short event traffic, not
part of that recurring total. At 9600, the interval from one control-frame
delimiter to the next frame's start is about 21.9 ms; at 38400 it is about
30.5 ms.

If the host loop wakes late, it sends one frame and schedules the next deadline
a full 33.3 ms after that actual send. It never emits a catch-up burst into the
firmware's reverse-direction window; missed slots are counted instead.

Startup obeys the same physical-wire rule. The normal runtime lets its
three-frame E-stop plus DISARM/HELLO prelude drain at the selected baud before
the first control, then preserves the next 33.3 ms control-start deadline
across the HELLO-to-connected transition. The calibration tool likewise drains
its three E-stop frames before starting the timed neutral stream. This avoids
mistaking fast RFCOMM/OS `write()` returns for bytes already transmitted by the
HC-06 UART.

The normal runtime also projects the 8N1 drain time of every accepted host
write. Urgent E-stop bursts are still queued immediately, but a control that
would have sat behind that event traffic is deferred until the projected UART
queue is empty. Its following deadline is then anchored a full 33.3 ms later.
Under this conservative software projection, an event lengthens one interval
instead of intentionally compressing the next into the firmware response
window. The logic-analyzer qualification described below remains required.

The Uno link uses NeoSWSerial and explicit receive/transmit windows. Its
software-UART transmitter still masks interrupts for each transmitted
character, so a short frame is not equivalent to interrupt-free full duplex.
Control and E-stop receive work has priority over periodic status.

The implementation pumps at most one HC-06 transmit byte per main-loop pass,
and only when:

- the receive parser is between complete frames;
- no enabled sonar group is between trigger and completed/expired echo
  capture;
- in the normal image, a valid control frame established the current period,
  at least 10 ms has elapsed since it completed, and at least 2 ms remains
  before the next frame (the windows end at about 19.9 ms at 9600 and 28.5 ms
  at 38400);
- no motor-board query is outstanding and D0/D1 has no unread byte;
- both the next motor-command and encoder-query deadlines are more than 2 ms
  away; and
- while the arm is enabled, Timer1 is in the 10–18 ms quiet portion of its
  20 ms servo frame.

A queued status may therefore be deferred and repeated state changes
coalesced rather than blocking the control path. A 14-byte status can span
more than one permitted window at 9600; control and E-stop receive still take
priority. Unbounded logging must never share the HC-06 UART.

These timing rules are a software design constraint, not hardware
qualification. Verify both baud modes with a logic analyzer while exercising
30 Hz control, 2 Hz status, immediate state changes, motor queries, and servo
movement, plus sonar capture when that flag is enabled. Any dropped/late
control, corrupted motor reply, pulse jitter, distorted sonar echo, or
watchdog edge is a release blocker.

## Uno resource audit

The v2 baseline was rebuilt from commit `ba4b8ad` with the same pinned
PlatformIO/AVR toolchain. The v3 numbers below come from the current build
matrix. “Static free” is 2048 bytes minus `.data` and `.bss`; it is not a
measurement of live stack headroom.

| Image | Flash | Static SRAM | Static free |
| --- | ---: | ---: | ---: |
| v2 closed-loop robot (`ba4b8ad`) | 31,668 B (98.2%) | 1,812 B (88.5%) | 236 B |
| v3 robot, unqualified default | 19,794 B (61.4%) | 1,367 B (66.7%) | 681 B |
| v3 qualified closed-loop robot | 29,226 B (90.6%) | 1,371 B (66.9%) | 677 B |
| v3 qualified robot + sonar | 31,252 B (96.9%) | 1,391 B (67.9%) | 657 B |
| v2 calibration (`ba4b8ad`) | 31,844 B (98.7%) | 1,835 B (89.6%) | 213 B |
| v3 calibration | 22,624 B (70.1%) | 1,465 B (71.5%) | 583 B |

The fair production comparison is v2 closed-loop against v3 qualified
closed-loop: flash falls by 2,442 bytes (7.7%), static SRAM by 441 bytes
(24.3%), and static free SRAM rises from 236 to 677 bytes. The unqualified
default is intentionally not used for that comparison because link-time
optimization can remove motion paths made unreachable by calibration gates.
For calibration, flash falls by 9,220 bytes (29.0%) and static SRAM by
370 bytes (20.2%).

The linked v3 matrix contains no application-visible `malloc`, `calloc`,
`realloc`, `free`, or C++ allocator symbols. The 256-byte live watermark gate
still applies because static free SRAM does not include worst-case call
frames, interrupt nesting, or library stack use. The sonar-enabled production
variant has only 1,004 bytes of flash remaining; keep that build in CI and
treat any material growth as a resource review trigger.

## Motor-board UART contract

The motor board is independent of the HC-06 host protocol:

- `$Car:` carries four A/B/C/D closed-loop speed targets in m/s.
- `$Car_Pwm:` carries four signed open-loop percentage targets.
- `$MOTOR_4CH_READ:encoder_20ms!` requests encoder increments.
- encoder-total requests are serialized through the same parser.
- DISARM, link loss, E-stop, disabled drive, and drive faults converge on
  `$Car:0,0,0,0!`.

The driver-board UART remains compiled and allocated in shipped Uno images.
`ROBOT_DRIVE_ENABLED=0` prevents initialization, polling, and commands; it does
not replace the concrete UART driver with a Null object.

## Authoritative motor-board reference

The ignored local vendor examples were read as the motor-board authority; they
were not copied or committed. Relevant snapshot hashes:

| Local reference | SHA-256 | What it establishes |
| --- | --- | --- |
| `examples/UART历程/驱动电机/uart_test/uart_test.ino` | `290154331d2b73dbf90639d3d9f0672b0165e2118464a8ffb5ccd1c7700a80c5` | init spelling, `$Car:`, `$Car_Pwm:`, A/B/C/D examples |
| `examples/UART历程/回读数据/uart_test/uart_test.ino` | `e9b35df163a3640f1e9c25a7fc7af5e3d292026ec6a87bed4793b23dfb09aa14` | query and reply spelling/field widths |
| `examples/C25 v5/User/app_motor.c` | `6b2ab0ed6731dacab09db950f3e6c889f490302317d98456100d5681b0efb406` | `$Car:` values are m/s wheel targets; controller owns PID/PWM |
| `examples/C25 v5/User/app_motor.h` | `98b8ebd4293a4ebc8f8f5128ee28b5cd8b356f58899a3b47946d1f5d65dad162` | 50 Hz PID, 60 mm wheel, 4680 counts, 160/170 mm geometry |
| `examples/C25 v5/User/Components/y_global/y_global.c` | `fa295dcbea4daa022c97cbddea3559eea51c35fba4345dacf03d3f09ff155083` | `$Car:` parser and `cmdOk` noise |

The Uno implementation replaces the examples' `String`, blocking reads,
delays, and response waits with bounded state machines. That changes scheduling
mechanics, not the board command syntax. Wheel order, signs, counts, geometry,
and exposed encoder timing remain calibration results rather than assumptions.
