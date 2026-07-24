# Calibration manual

Written for someone touching this robot for the first time. Every step gives
the exact command to type and the exact file and variable to edit. You do not
need to read any firmware source code.

There is **one** calibration firmware (`calibration`) and **one** interactive
tool (`warehouse-robot calibrate`). The operations available in that session
depend on the same feature flags as the normal `robot` image. For example,
drive commands are rejected when `ROBOT_DRIVE_ENABLED=0`; calibration mode
does not bypass a disabled subsystem.

## Checklist

Print this or keep it open. Do the chapters in order; tick as you go.

- [ ] 0.1 Wheels raised off the ground, servo/motor power available
- [ ] 0.2 Host link connected to A4/A5 (HC-06 or USB-TTL adapter)
- [ ] 1.1 Calibration firmware flashed (`pio run -t upload -e calibration`)
- [ ] 1.2 Session connected (`warehouse-robot calibrate --port ...`)
- [ ] 2.1 Arm neutral positions marked (j0/j1/j2 center, j3 open+closed)
- [ ] 2.2 Arm direction signs recorded (`dir j0..j3`), j1+j2 show `synced`
- [ ] 2.3 Arm travel limits marked (j0/j1/j2 lower+upper), base span ≥ 180°
- [ ] 3.1 Motor mapping and direction recorded (`motor 0..3`)
- [ ] 3.2 Encoder mapping and direction recorded (`enc 0..3`)
- [ ] 3.3 Wheel geometry measured and recorded (`geom ...`)
- [ ] 4.2 Closed-loop verification passed after write-back/reflash
- [ ] 4.1 `export` block pasted into `src/app/BuildConfig.h`
- [ ] 4.2 Values re-checked with a reflashed calibration build
- [ ] 4.3 Worst-case stack-watermark run reports at least 256 bytes free
- [ ] 5.1 Matching `*_CALIBRATED=1` flags set only for the enabled features
      whose procedures passed; robot firmware flashed

---

## Chapter 0 — Before you start

### 0.1 What the words mean

- **Front of the robot**: the side the gripper points at when the arm faces
  forward. "Forward" for a wheel means the rotation that would move the robot
  toward its front.
- **The four wheels** are always named in this order:
  `fl` front-left, `fr` front-right, `rl` rear-left, `rr` rear-right
  (left/right as seen standing behind the robot, looking at its front).
- **Board channel** 0–3: the four motor sockets (M1–M4) on the UART motor
  driver board. Part of chapter 3 is discovering which channel is which wheel
  — you do not need to know this in advance.
- **Joint numbers**: j0 base (rotates the whole arm), j1 shoulder,
  j2 elbow, j3 gripper.

### 0.2 Wiring for a calibration session

The Uno's D0/D1 hardware serial belongs to the **motor board**. The computer
never talks over it. The computer talks over the **host link** on A4/A5:

| Connection | Where |
| --- | --- |
| Motor board TX → Uno D0, RX ← Uno D1 | needed for chapter 3 (motors) |
| Host link (HC-06 Bluetooth **or** USB-TTL adapter): TX → Uno A5, RX ← Uno A4 (3.3 V-safe divider/level shifter) | needed for the whole session |
| USB cable | only for flashing firmware |

Rules:

- **Unplug the D0/D1 motor-board wires while uploading firmware over USB**,
  then reconnect them.
- Put the robot on a stand so **all four wheels spin freely in the air**.
  Chapter 3 spins motors for real.
- The HC-06 must be in normal data mode. The project and module default is
  9600 baud; use `--baud 9600` in step 1.2.
- 38400 is optional. Changing `ROBOT_HOST_BAUD` or the host's `--baud` does
  not configure the module. Complete and verify the isolated
  [HC-06 38400 procedure](hc-06-38400.md), then select 38400 at all three
  endpoints.

## Chapter 1 — Flash and connect

### 1.1 Flash the calibration firmware

From the repository root, with the D0/D1 wires unplugged:

```sh
pio run -t upload -e calibration
```

Reconnect the motor-board wires afterwards.

### 1.2 Start the session

```sh
warehouse-robot list-ports
# Bluetooth on macOS may look like /dev/cu.HC-06...; a USB-TTL adapter looks
# like /dev/cu.usbserial-...; Windows shows COM ports.
warehouse-robot calibrate --port /dev/cu.HC-06 --baud 9600
```

Notes:

- Opening an HC-06 RFCOMM port does **not** reset the Uno. The tool waits for
  the serial link to settle, sends E-stop, waits for that burst to drain at
  the selected HC-06 baud, supplies 0.7 s of neutral 30 Hz control, clears a
  prior E-stop back to the fail-closed DISARMED state, and verifies a
  protocol-v3 `calibration` HELLO before showing the prompt.
- After power-up all servos are limp (the firmware never moves them on its
  own); the arm sags under gravity. Hold it or rest it on a support.
- The **first** command sent to each joint attaches it at the raw 90° anchor;
  both host and firmware reject any other first target. Hold and align the
  joint before sending `90`. Every later command slews at 60°/s.
- Type `help` any time for the full command list; `q` quits.
- Protocol-v3 calibration traffic is request/reply. Encoder and sensor reports
  are returned only when the tool explicitly asks for them; there is no
  background detailed telemetry stream.

## Chapter 2 — Calibrate the arm

Work in the order shoulder → elbow → base → gripper. The firmware has a
built-in shoulder/elbow anti-collision guard; if a command is refused with
"coupling guard", that is protection working — move the *other* joint first
and continue.

### 2.1 Neutral (center) positions

#### 2.1.1 Shoulder and elbow

1. `j1 90`, hold the arm, let it settle. Nudge with `j1 +1` / `j1 -1` until
   the **upper arm is exactly vertical** (check against a square or plumb
   line). Then `mark j1 center`.
2. `j2 90`. Nudge until the **forearm is exactly level with the ground**.
   Then `mark j2 center`.

#### 2.1.2 Base and gripper

1. `j0 90`, nudge until the arm points **straight ahead**, `mark j0 center`.
2. Hold the gripper clear and send `j3 90` once to establish its required
   first-command anchor. Then send `j3 80`, nudge until the gripper is
   **fully open without straining**,
   `mark j3 open`. Close it slowly (`j3 -2` at a time) until the fingers just
   touch, `mark j3 closed`.

**Write-back (done for you):** these four/five numbers become
`BaseZeroDegrees`, `ShoulderZeroDegrees`, `ElbowZeroDegrees`,
`GripperOpenDegrees`, `GripperClosedDegrees` in `src/app/BuildConfig.h` —
the `export` command in chapter 4 prints them ready to paste.

### 2.2 Direction signs

The conventions (memorize the bold parts, they define "+1"):

| Joint | Positive direction means | Test | Record |
| --- | --- | --- | --- |
| j1 shoulder | raw angle up ⇒ upper arm tilts **up/back** (toward the robot's rear) | `j1 +5`, watch it, `j1 -5` | tilts back → `dir j1 +1`; tilts front → `dir j1 -1` |
| j2 elbow | raw angle up ⇒ forearm tilts **up** | `j2 +5`, watch, `j2 -5` | up → `dir j2 +1`; down → `dir j2 -1` |
| j0 base | raw angle up ⇒ arm swings to the robot's **right** | `j0 +10`, watch, `j0 -10` | right → `dir j0 +1`; left → `dir j0 -1` |
| j3 gripper | going **down** in raw angle from open **closes** the fingers | you saw this in 2.1.2 | closed mark < open mark → `dir j3 +1`; otherwise `dir j3 -1` |

A servo needing `-1` is a normal mirror-mounted installation, not a wiring
mistake.

> **Important:** once a joint has both a center mark and a `dir`, the session
> pushes them into the firmware and prints `synced j1 ...`. Until you see
> **both j1 and j2 synced**, the anti-collision guard is running on default
> assumptions and can block safe poses or allow unsafe ones — move slowly and
> keep a hand on the arm. `s` shows the sync state.

**Write-back:** the four signs become `ServoDirectionSign[4]` in
`src/app/BuildConfig.h` (printed by `export`).

### 2.3 Travel limits

These are conservative **usable** travel limits, recorded as raw angles. Do
not drive a servo into a hard stop or use audible strain as a measuring tool.
Support the linkage, keep independent servo power cutoff within reach, and
move in the smallest available increments near each boundary.

1. **Shoulder range (elbow at its center):** `j2 90` (or your marked elbow
   center). Approach the expected boundary in small steps, switch to 1° steps,
   and stop before contact, cable tension, binding, or strain. Back off to a
   repeatably clear pose and `mark j1 upper`. Repeat toward the other end for
   `mark j1 lower`. A **coupling-guard block** is a valid safety boundary for
   that shoulder/elbow combination; never push through it to seek a servo stop.
2. **Elbow range (shoulder at its center):** `j1 90` (marked center). Sweep
   `j2` with the same small-step and clearance rules. Because the safe elbow
   boundary changes with shoulder angle, check several supported,
   collision-clear shoulder poses and record limits that remain clear
   throughout the intended workspace. Do not defeat the coupling guard.
3. **Base range:** sweep `j0` to both physical ends, `mark j0 lower` /
   `mark j0 upper`, stopping before cable tension or contact. If the intended
   design requires 180° but the safe measured span is smaller, fix the
   mechanical routing before widening the recorded values.
4. Gripper limits are derived automatically from the open/closed marks.

**Write-back:** `ServoLowerDegrees[4]` and `ServoUpperDegrees[4]` in
`src/app/BuildConfig.h` (printed by `export`).

## Chapter 3 — Calibrate the drive motors

Everything here happens with the **wheels off the ground**. Motor spins run
only while DISARMED, are speed-capped (100 %, 200 mm/s), and stop by
themselves after at most 10 seconds; `stop` ends one early.

This chapter also requires `ROBOT_DRIVER_ENABLED=1` and
`ROBOT_DRIVE_ENABLED=1`. If a spin command is refused with "invalid state" or
"disabled", check those flags before checking motor power and D0/D1 wiring.

### 3.1 Which motor is which wheel, and which way is forward

For each board channel 0, 1, 2, 3:

1. `m0 30 2` — channel 0 spins at 30 % for 2 seconds.
2. Watch: **which wheel moved**, and did it turn **forward** (the direction
   that would drive the robot toward its front) or backward?
3. Record it, e.g. the front-left wheel moved forward:

   ```text
   motor 0 fl fwd
   ```

   or the rear-right wheel moved backward: `motor 0 rr rev`.
4. Repeat with `m1 30 2`, `m2 30 2`, `m3 30 2` until all four wheels are
   accounted for. `s` shows what you have recorded.

**Write-back:** `MotorCommandMap[4]` and `MotorCommandSign[4]` in
`src/app/BuildConfig.h` (printed by `export`).

### 3.2 Which encoder is which wheel, and which way counts up

1. Type `counts`. Each invocation requests two protocol-v3 drive pages: raw
   increment/total values indexed by **motor-board channel**, then measured
   speed indexed by compiled logical wheel (`fl/fr/rl/rr`). Before write-back,
   use the raw board-channel table to discover encoder wiring; the logical
   speed table still reflects the mappings currently compiled into the Uno.
   If a report is unavailable, wait for the board query to complete and retry.
2. Spin one wheel: `m0 30 3`, then immediately type `counts` once or twice
   while it spins. Exactly one channel should show a clearly non-zero
   increment — that encoder channel belongs to the wheel you saw moving.
3. Was the wheel turning **forward** while the increment was **positive**?
   Then it counts up going forward. Record, e.g.:

   ```text
   enc 2 fl fwd
   ```

   If the increment was negative while the wheel went forward (or positive
   while it went backward): `enc 2 fl rev`.
4. Repeat for the other three channels/wheels.

Tip: if the wheel you spin moved *backward* (you saw that in 3.1), flip the
reading in your head: backward motion with a negative count still means
"counts up going forward" → `fwd`.

**Write-back:** `EncoderChannelMap[4]` and `EncoderDirectionSign[4]` in
`src/app/BuildConfig.h` (printed by `export`).

### 3.3 Wheel geometry

Four numbers, measured with a ruler/calipers and the encoder totals:

1. **Wheel diameter (mm):** measure across a wheel, e.g. 60.
2. **Counts per revolution:** motors stopped. Type `counts` and write down
   one channel's `total`. Rotate that wheel **by hand exactly 10 full
   turns** in one direction. `counts` again; counts-per-rev =
   (new total − old total) ÷ 10, sign dropped, e.g. 4680.
3. **Track (mm):** distance between the left and right wheel centerlines.
4. **Wheelbase (mm):** distance between the front and rear axle centerlines.

Record all four in one line:

```text
geom 60 4680 160 170
```

**Write-back:** `WheelDiameterMm`, `EncoderCountsPerRevolution`,
`WheelTrackMm`, `WheelbaseMm` in `src/app/BuildConfig.h` (printed by
`export`).

### 3.4 Defer closed-loop verification until after write-back

The `motor`/`enc` records live in the Python session until `export`; they do
not alter the running firmware. Complete chapter 4.1 and reflash before using
the logical speed table for the final closed-loop check.

## Chapter 4 — Write everything back

### 4.1 Export and paste

Type `export`. It prints one block containing **every number you measured**
— arm centers, arm limits, arm directions, motor maps, encoder maps,
geometry:

```cpp
// Paste over the matching lines in src/app/BuildConfig.h
constexpr uint8_t BaseZeroDegrees = 92;
...
constexpr uint16_t WheelbaseMm = 170;
```

Open `src/app/BuildConfig.h`, search for `Per-joint servo calibration` (the
arm lines, with the five `...Degrees` constants just above the comment) and
`Per-wheel drive calibration` (the motor lines right below), and **replace
the matching constants line by line**. No other file needs editing —
everything else reads these constants.

If the export ends with a `// WARNINGS:` block, something was never
measured (a default was used) — go back and finish it before moving on.

### 4.2 Double-check

```sh
pio test -e native                     # geometry consistency checks
pio run -t upload -e calibration       # unplug D0/D1 first, reconnect after
```

Reconnect, restart `warehouse-robot calibrate`, send each joint to 90 and
confirm the physical pose matches the meaning of each center (upper arm
vertical, forearm level, arm straight ahead).

With all wheels raised, verify the newly compiled drive mapping and geometry.
For each board channel:

1. `v0 150 3` asks channel 0 to hold 150 mm/s for 3 seconds.
2. While it spins, run `counts`. In the second table, find the logical wheel
   that channel is compiled to drive; its measured speed should be about
   +150 mm/s (roughly ±25%). Repeat with `v0 -150 3`.
3. Repeat for channels 1–3. A near-zero, wrong-sign, or wildly wrong value
   means chapter 3 data is wrong; fix, export, and reflash again.

### 4.3 Qualify the 256-byte stack reserve

The calibration image includes an on-demand stack-watermark instrument. It is
not present in the normal `robot` protocol.

1. Reset or power-cycle the Uno immediately before the run. The firmware paints
   its canary once, after initialization; a new run needs a new paint.
2. Establish the required `j0 90` ... `j3 90` anchors. Queue conservative,
   collision-clear moves for the four joints, start a bounded raised-wheel
   motor spin, and immediately run `stress 10`. That command sends neutral
   compact control frames with a 30 Hz target while the firmware continues
   servo, motor, encoder, and sonar work. It prints the actual send rate,
   maximum interval, and missed slots; require zero missed slots and
   investigate anything materially below 30 Hz before accepting the run.
3. Request `counts`, `sensors`, and `s` once to cover each largest
   calibration reply, then type `stack` (or `system`).
4. Require output in the form
   `minimum untouched stack: <N> B — PASS (required >= 256 B)`. Any `FAIL`,
   timeout, reset, or value below **256 bytes** blocks release.
5. Repeating `stack` without a reset reports the lifetime minimum since the
   canary was painted, so read it again after the longest/highest-load phase.

The reported number already treats a deliberate 48-byte unpainted gap as used.
The scan briefly masks interrupts so the current read cannot race a deeper ISR
stack entry; it is a conservative lower bound after
`RobotApplication::begin()`. A simple manual sequence that does not combine
the listed loads is diagnostic only.
Stack PASS also does not qualify UART collisions or servo jitter; capture those
with a logic analyzer as described in
[the protocol scheduling section](protocol-v3.md#scheduling-and-hc-06-coexistence).

## Chapter 5 — Mark calibration complete

Each enabled actuator requires its corresponding calibration evidence before
it may move. Open `platformio.ini`, find `[env:robot]`, and set only the flags
whose procedures you completed:

```ini
    -DROBOT_ARM_ENABLED=1
    -DROBOT_ARM_CALIBRATED=1
    -DROBOT_DRIVE_ENABLED=1
    -DROBOT_DRIVE_CALIBRATED=1
```

`ROBOT_DRIVE_ENABLED=0` is the supported arm-only normal-firmware test
configuration: the concrete UART driver still compiles and consumes its fixed
memory, but the firmware does not initialize, poll, or command the driver
board. Physically isolate motor power as well. Select exactly one of
`ROBOT_DRIVER_CONTROL_CLOSE` and `ROBOT_DRIVER_CONTROL_OPEN`; changing that
selection does not waive drive calibration.

Flash it (D0/D1 unplugged, then reconnected):

```sh
pio run -t upload -e robot
```

What to expect on the robot build: servos stay limp until the first ARM,
then drive to the stow pose — pose the arm near stow by hand before ARMing
so it doesn't jump. DISARM does not release the servos. After a fault latch,
send ClearFault before ARMing again.

## Chapter 6 — Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `blocked: shoulder/elbow coupling guard...` | Protection, not a fault. Back off, or move the other joint first (chapter 2.3) |
| Guard behaves backwards (blocks obviously-safe poses / allows deep folds) | j1/j2 not yet synced, or a wrong `dir`. Mark center + set dir for both, watch for `synced j1`/`synced j2`, re-test the direction if it persists |
| `invalid state` or `disabled` on `j`/`m`/`v` commands | Wrong firmware (must be `calibration`), not DISARMED, or the matching `ROBOT_ARM_ENABLED` / `ROBOT_DRIVE_ENABLED` flag is `0`; for motor commands also check board power and D0/D1 |
| Motor spin acknowledged but nothing turns | Motor power supply off, or that channel has no motor plugged in |
| `counts` never returns a drive report | `ROBOT_DRIVE_ENABLED=0`, motor-board D0/D1 disconnected, or board unpowered |
| Arm goes limp / robot resets when you plug USB | Expected: USB opening resets the Uno. Do the whole session over the A4/A5 link; USB is only for flashing |
| `serial device ... is not available` | Wrong port name; rerun `warehouse-robot list-ports` — macOS needs the full `/dev/` path |
| Nothing responds at all | Host-link baud mismatch. The project and HC-06 default is 9600; firmware, module, and `--baud` must match. For 38400, complete the linked isolated procedure first |
| A joint jumps hard on its first command | First command per joint is never slewed; make it `90` and hold the arm |
| Closed-loop `v` speed far from commanded | Wrong `enc` sign/channel or wrong `geom` numbers; redo 3.2/3.3 |

## Appendix — why the elbow guard exists

The elbow servo drives the forearm through a parallelogram linkage, so it
sets the forearm's angle relative to the ground, not relative to the upper
arm. The rod-to-upper-arm clearance is g = 40·sin(fold + 20°) with 10 mm
links: bare contact at a 145.5° fold, hard limit 140°, running firmware
135°, calibration band [−5°, 138°]. The fold angle only depends on the
wrist-to-shoulder distance (d = 240·cos(fold/2)), which is why the running
firmware projects every arm target onto an annular reachable region and
calibration never needs to reserve margin for it manually.
