# Calibration manual

Written for someone touching this robot for the first time. Every step gives
the exact command to type and the exact file and variable to edit. You do not
need to read any firmware source code.

There is **one** calibration firmware (`calibration`) and **one** interactive
tool (`warehouse-robot calibrate`). Together they calibrate the arm servos
**and** the drive motors in a single session.

## Checklist

Print this or keep it open. Do the chapters in order; tick as you go.

- [ ] 0.1 Wheels raised off the ground, servo/motor power available
- [ ] 0.2 Host link connected to A4/A5 (HC-05 or USB-TTL adapter)
- [ ] 1.1 Calibration firmware flashed (`pio run -t upload -e calibration`)
- [ ] 1.2 Session connected (`warehouse-robot calibrate --port ...`)
- [ ] 2.1 Arm neutral positions marked (j0/j1/j2 center, j3 open+closed)
- [ ] 2.2 Arm direction signs recorded (`dir j0..j3`), j1+j2 show `synced`
- [ ] 2.3 Arm travel limits marked (j0/j1/j2 lower+upper), base span ≥ 180°
- [ ] 3.1 Motor mapping and direction recorded (`motor 0..3`)
- [ ] 3.2 Encoder mapping and direction recorded (`enc 0..3`)
- [ ] 3.3 Wheel geometry measured and recorded (`geom ...`)
- [ ] 3.4 Closed-loop verification passed (`v<n>` speeds match)
- [ ] 4.1 `export` block pasted into `src/app/BuildConfig.h`
- [ ] 4.2 Values re-checked with a reflashed calibration build
- [ ] 5.1 `ROBOT_ARM_CALIBRATED=1` and `ROBOT_DRIVE_CALIBRATED=1` set in
      `platformio.ini`, robot firmware flashed

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
| Host link (HC-05 Bluetooth **or** USB-TTL adapter): TX → Uno A5, RX ← Uno A4 (3.3 V divider) | needed for the whole session |
| USB cable | only for flashing firmware |

Rules:

- **Unplug the D0/D1 motor-board wires while uploading firmware over USB**,
  then reconnect them.
- Put the robot on a stand so **all four wheels spin freely in the air**.
  Chapter 3 spins motors for real.
- The HC-05 must be in normal data mode and slave role. The default host-link
  baud is 38400. If its data mode is configured for 9600, open
  `platformio.ini`, find `[uno]`, and uncomment the one shared
  `-DROBOT_HOST_BAUD=9600UL` line. Then use `--baud 9600` in step 1.2.
  The 38400 baud commonly used for full HC-05 AT-command mode does not prove
  that normal data mode is also set to 38400.

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
# Bluetooth on macOS looks like /dev/cu.HC-05...; a USB-TTL adapter looks
# like /dev/cu.usbserial-...; Windows shows COM ports.
warehouse-robot calibrate --port /dev/cu.HC-05
# add --baud 9600 if you set the 9600 line in chapter 0.2
```

Notes:

- After power-up all servos are limp (the firmware never moves them on its
  own); the arm sags under gravity. Hold it or rest it on a support.
- The **first** command sent to each joint snaps that servo straight to the
  target (every later command slews at 60°/s). So the first command to every
  joint is always `90`, holding the arm steady.
- Type `help` any time for the full command list; `q` quits.

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
2. `j3 80`, nudge until the gripper is **fully open without straining**,
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

These are the servo's **absolute** mechanical ends, recorded as raw angles.

1. **Shoulder range (elbow at its center):** `j2 90` (or your marked elbow
   center). Step `j1 +5` repeatedly until a hard mechanical stop or audible
   strain, back off `j1 -2`, `mark j1 upper`. Repeat toward the other end
   for `mark j1 lower`. If the session reports a **coupling-guard block**
   instead of a hard stop, that end is not the shoulder's true limit — tilt
   the elbow to a compatible angle, keep pushing the shoulder, and only mark
   at a real mechanical stop.
2. **Elbow range (shoulder at its center):** `j1 90` (marked center). Sweep
   `j2` the same way. Caveat: with the shoulder vertical, the **downward**
   sweep usually hits the coupling guard before the servo's own end — that
   guard boundary moves with the shoulder and must **not** be recorded as
   `lower`. Tilt the shoulder well forward first, then sweep the elbow down
   to its hard stop and `mark j2 lower`. Return the shoulder to vertical (or
   slightly back) for the upward sweep and `mark j2 upper`.
3. **Base range:** sweep `j0` to both physical ends, `mark j0 lower` /
   `mark j0 upper`. The design requires **at least 180°** of total base
   travel; `export` warns if you measured less (usually a snagged cable).
4. Gripper limits are derived automatically from the open/closed marks.

**Write-back:** `ServoLowerDegrees[4]` and `ServoUpperDegrees[4]` in
`src/app/BuildConfig.h` (printed by `export`).

## Chapter 3 — Calibrate the drive motors

Everything here happens with the **wheels off the ground**. Motor spins run
only while DISARMED, are speed-capped (100 %, 200 mm/s), and stop by
themselves after at most 10 seconds; `stop` ends one early.

If a spin command is refused with "invalid state", the motor board is not
powered or its D0/D1 wires are still unplugged.

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

1. Type `counts` to see the four encoder channels (increment, total,
   measured speed). If it says no telemetry, wait a second and retry.
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

### 3.4 Closed-loop verification (raised wheels)

This proves the motor board's own speed controller, your mappings, and your
geometry agree — it replaces the old separate "raised-wheel qualification"
firmware.

For each channel:

1. `v0 150 3` — channel 0 is asked to hold 150 mm/s for 3 seconds.
2. While it spins, `counts`: the **measured mm/s** column for that channel
   should read about 150 (within roughly ±25 %). Also try `v0 -150 3`.
3. If the measured speed is wildly off, near zero, or the wrong sign, one of
   3.1–3.3 is wrong for that wheel — re-measure it before continuing.

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

## Chapter 5 — Mark calibration complete

The robot firmware refuses to ARM until you declare the calibration done.
Open `platformio.ini`, find `[env:robot_closed_loop]`, and change two lines:

```ini
    -DROBOT_ARM_CALIBRATED=1
    -DROBOT_DRIVE_CALIBRATED=1
```

(Set only the one you actually completed; the arm stays parked without its
flag, ARM stays locked without the drive flag. If you use the open-loop
fallback, make the same change in `[env:robot_open_loop]`.)

Flash it (D0/D1 unplugged, then reconnected):

```sh
pio run -t upload -e robot_closed_loop
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
| `invalid state` on `j`/`m`/`v` commands | Wrong firmware (must be the `calibration` build), not DISARMED, or — for motor spins — the motor board is unpowered/disconnected |
| Motor spin acknowledged but nothing turns | Motor power supply off, or that channel has no motor plugged in |
| `counts` never shows telemetry | Motor board D0/D1 wires disconnected or board unpowered |
| Arm goes limp / robot resets when you plug USB | Expected: USB opening resets the Uno. Do the whole session over the A4/A5 link; USB is only for flashing |
| `serial device ... is not available` | Wrong port name; rerun `warehouse-robot list-ports` — macOS needs the full `/dev/` path |
| Nothing responds at all | Host-link baud mismatch: firmware default is 38400; an HC-05 set to 9600 data mode needs the shared `[uno]` flag uncommented **and** `--baud 9600` |
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
