# Arm calibration manual

Written for someone touching this project for the first time: every step
gives a copy-pasteable command and the exact file location to edit, with no
need to read the firmware source. Only prerequisite: PlatformIO and the
Python environment are installed per the [README](../README.md)
(`python3 -m pip install -e '.[test]'`), and the arm servos are wired up with
servo power available.

## 0. The theory in three sentences (skippable)

The elbow servo is mounted on the base and, through a parallelogram linkage,
controls the forearm's angle **relative to the ground**, not the elbow
joint's relative angle — so "how much further the elbow can turn" depends on
where the shoulder is. The firmware has a built-in anti-collision guard
(shoulder/elbow coupled fold-angle limit): pushing a joint too far during
calibration gets rejected or stopped by the firmware, it will not crash the
linkage. All you need to do is find the zero position, both travel ends, and
the direction for 4 joints — 16 numbers total — drop them into one file, and
reflash.

## 1. Flash the calibration firmware

Run from the repository root (if the Uno's D0/D1 pins are wired to the motor
driver board, unplug that link before uploading):

```sh
pio run -t upload -e arm_calibration
```

## 2. Find the serial port and start a calibration session

```sh
warehouse-robot list-ports
# macOS looks like /dev/cu.usbmodem14101, Windows looks like COM5
warehouse-robot calibrate --port /dev/cu.usbmodem14101
```

Notes:

- Opening the USB serial port resets the Uno; the tool automatically waits
  2 seconds for this, which is expected. This is also why you should
  **not** use the one-shot `calibrate-joint` command for fine-tuning — every
  invocation reopens the port, resets the board, and lets the servos go
  limp. The `calibrate` session keeps the port open for the whole run.
- Right after power-up all servos are limp (the firmware never moves them on
  its own), so the arm will sag under gravity. Hold it or let it rest on a
  support.
- The **first** command sent to each joint snaps that servo straight to the
  target angle (every command after that slews at 60°/s). So the first
  command to every joint should always be `90`, and you should be holding
  the arm steady when you send it.

In-session commands (type `help` any time to see this again):

| Command | Effect |
| --- | --- |
| `j2 95` | Move joint 2 to 95° (joint numbers: 0 base, 1 shoulder, 2 elbow, 3 gripper) |
| `j2 +2` / `j2 -1` | Nudge relative to the last commanded angle |
| `mark j2 center` | Record the current angle as that joint's center (options: lower/upper/center; the gripper uses open/closed instead) |
| `dir j1 -1` | Record the direction sign |
| `s` | Show each joint's commanded angle, firmware-reported angle, recorded marks, and the fold-angle estimate |
| `export` | Print a code block ready to paste into BuildConfig.h |
| `q` | Quit (servos hold their last position) |

## 3. Calibrate joint by joint

Work in the order **shoulder → elbow → base → gripper** (the elbow's guard
needs the shoulder to already have a position).

### 3.1 Shoulder (j1)

1. `j1 90`, hold the arm steady, let the servo settle.
2. Nudge with `j1 +1` / `j1 -1` until the upper arm is **vertical** (check
   with a square or by eye against a plumb line).
3. `mark j1 center`
4. Direction: send `j1 +5` and see whether the upper arm tilts toward the
   robot's **front**. If yes → `dir j1 +1`; if reversed → `dir j1 -1`. Then
   `j1 -5` to return to vertical.
5. Both ends: keep sending `j1 +5` until any of the following happens, then
   back off with `j1 -2` and `mark j1 upper`: (a) it hits a mechanical stop
   or the servo strains audibly, (b) the command is rejected by the
   firmware, (c) the session reports the fold angle approaching 138°. Do the
   same in the other direction for `mark j1 lower`.

### 3.2 Elbow (j2)

1. First return j1 to its marked center (`j1 90`, or whatever value you
   marked nearby).
2. `j2 90`, hold the arm steady.
3. Nudge until the forearm is **level**. Check: the servo's white horn
   should now sit at roughly 20° from horizontal.
4. `mark j2 center`
5. Direction: send `j2 +5`; if the forearm **tilts up**, `dir j2 +1`,
   otherwise `-1`.
6. Both ends as with the shoulder; `s` shows the fold-angle estimate (the
   guard hard-stops at 138°, so being blocked there is expected and just
   means that end of travel has been reached — back off 2° and record it).

### 3.3 Base (j0)

1. `j0 90`, nudge until the arm points **straight ahead**, `mark j0 center`.
2. Direction: `j0 +10`; if the arm swings toward the robot's **right**,
   `dir j0 +1` (preset semantics: 0°=left, 90°=front, 180°=right), otherwise
   `-1`.
3. Record lower/upper at both physical ends. The mechanical design requires
   **at least 180°** of total travel; if `export` warns that it's under
   180°, check whether a cable is snagging the rotation.

### 3.4 Gripper (j3)

1. `j3 80`, nudge until it is **fully open but not straining**,
   `mark j3 open`.
2. Close it slowly (`-2` at a time) until the fingers just touch,
   `mark j3 closed`.
3. Direction: if moving to a **smaller** angle from open is the closing
   direction, `dir j3 +1`; if it takes a larger angle to close, `dir j3 -1`.

## 4. Export and write back to the firmware

Type `export` in the session; it prints something like:

```cpp
// Paste over the matching lines in src/app/BuildConfig.h
constexpr uint8_t BaseZeroDegrees = 92;
constexpr uint8_t ShoulderZeroDegrees = 88;
constexpr uint8_t ElbowZeroDegrees = 95;
constexpr uint8_t GripperOpenDegrees = 78;
constexpr uint8_t GripperClosedDegrees = 24;

constexpr uint8_t ServoLowerDegrees[4] = {2, 15, 20, 19};
constexpr uint8_t ServoUpperDegrees[4] = {178, 165, 170, 83};
constexpr int8_t ServoDirectionSign[4] = {1, -1, 1, 1};
```

Open `src/app/BuildConfig.h`, search for `Per-joint servo calibration`, and
**replace the matching constants line by line** (the five single-value
`...Degrees` constants sit just above the comment block; the three arrays
follow right below it). No other file needs to change — `RuntimeConfig`'s
defaults reference these constants directly.

If `export` ends with a `// WARNINGS:` block, it means some values were
never marked (defaults were used) or the base travel is insufficient — go
back to the session and finish measuring before moving on; don't carry
warnings into the next step.

## 5. Verify and enable the arm

```sh
pio test -e native        # test_arm_joint_safety checks geometry consistency
pio run -t upload -e arm_calibration   # reflash and double-check each center is still accurate
```

Once confirmed, open `platformio.ini`, find `[env:uart_closed_loop_robot]`,
and **add one line** at the end of its `build_flags`:

```ini
[env:uart_closed_loop_robot]
extends = uno
build_flags =
    ${uno.build_flags}
    -DROBOT_BACKEND_UART=1
    -DROBOT_UART_CLOSED_LOOP=1
    -DROBOT_DRIVE_QUALIFICATION=0
    -DROBOT_DRIVE_CALIBRATION_QUALIFIED=0
    -DROBOT_ARM_CALIBRATION=0
    -DROBOT_ARM_CALIBRATED=1
```

Rebuild and reflash that environment. The production firmware **will not**
move the arm on power-up; the servos only energize and drive to the stow
pose the first time it's ARMed — before ARMing, manually pose the arm near
its stow position to avoid a jump. DISARM does not release the servos (the
arm won't drop); after a fault latch, send ClearFault before ARMing again.

## 6. Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| Command replies `blocked by the shoulder/elbow coupling guard` | Hit the shoulder/elbow coupling guard (fold angle outside [-5°, 138°]) — this is the protection working, not a fault; back off, or move the other joint first |
| Command replies `invalid state` | The board isn't running the `arm_calibration` firmware, or it isn't DISARMED (the session sends DISARM automatically on start — restart the session) |
| Arm stops short of the target mid-slew | Same guard; check `s` for the fold-angle estimate |
| Arm goes limp then snaps after every command | You're using the one-shot `calibrate-joint` command; switch to the `calibrate` session |
| `serial device ... is not available` | Wrong port name; rerun `warehouse-robot list-ports` — macOS requires the full `/dev/` path |
| A joint jumps hard on its first command | The first command to a joint is never slewed; hold the arm steady and make the first command close to the joint's actual current position |
| `s` always shows `-` for telemetry | Telemetry arrives roughly once a second, so wait a moment; if it never appears, confirm the firmware is actually the `arm_calibration` build |

## Appendix: theoretical background (for the curious)

The parallel clearance between the connecting rod and the upper arm is
g = 40·sin(fold angle + 20°). With 10 mm-wide links, bare contact occurs at
145.5°; allowing for nut/washer clearance brings that to 140°, and the
running firmware tightens it further to 135° for drivetrain-stiffness
margin (calibration mode relaxes this to 138°). The fold angle is
determined solely by the wrist point's distance from the shoulder axis
(d = 240·cos(fold angle / 2)), which is why the running firmware implements
it as an annular reachable region and automatically projects every target
onto it — that's the reason calibration doesn't need to manually reserve
margin for the coupling.
