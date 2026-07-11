# Warehouse Robot Control System

Arduino Uno firmware and a Python gamepad controller for a four-wheel mecanum
warehouse robot with a three-axis arm, gripper, and directional ultrasonic
sensors. The runtime is deliberately cooperative and non-blocking; it does not
use an RTOS or dynamic allocation.

## Architecture

The firmware processes one immutable control frame through independent state
machines:

```text
Bluetooth/USB -> Communication -> OperatorControlFrame
                                      |
Ultrasonic -> SensorSnapshot -> Assist + Safety -> final DriveIntent
                                      |              |
                                ArmSubsystem    ChassisSubsystem
                                                       |
                                             selected DriveBackend
```

- `src/domain`: hardware-independent commands, state, mecanum mixing, and arm
  inverse kinematics.
- `src/subsystems`: communication mailbox, chassis, arm planner, sensors,
  ranging assist, and safety arbitration.
- `src/drivers`: interchangeable L293D, UART encoder-board, and null backends.
- `src/app`: build profile, pins, calibration console, and system composition.
- `scripts/robot_control`: Python input, mapping, protocol, transport, and CLI.

Communication and sensors never call actuators directly. The safety supervisor
has final authority over the chassis. A lost command link stops the chassis
after 300 ms while the arm and gripper hold their last targets.

## Build profiles

```sh
pio run -e l293d_dev
pio run -e uart_encoder_robot
pio run -e arm_calibration
pio test -e native
```

`l293d_dev` keeps the open-loop shield and newline diagnostic commands:

```text
C:<forward>,<turn>,<strafe>
W:<front_left>,<front_right>,<rear_left>,<rear_right>
M:<motor_index>,<speed>
```

`uart_encoder_robot` is the final robot profile. The motor board occupies the
Uno hardware UART at 115200 baud and uses its `$Car:...!`/readback protocol.
Bluetooth on A5/A4 carries compact, versioned COBS frames at 9600 baud.

`arm_calibration` disconnects the drive backend and accepts USB commands such
as `J:0,90` for base, `J:1,90` for shoulder, `J:2,90` for elbow, and `J:3,60`
for the gripper.

## UART robot wiring

| Function | Uno pin |
| --- | --- |
| Motor-board TX -> Uno RX | D0 |
| Motor-board RX <- Uno TX | D1 |
| Bluetooth TX -> Uno RX | A5 |
| Bluetooth RX <- Uno TX | A4 through a 3.3 V-safe divider |
| Base / shoulder / elbow / gripper servo | D3 / D5 / D6 / D9 |
| Shared ultrasonic trigger | D2 |
| Front pair echoes | D4 / D7 |
| Left pair echoes | D8 / D10 |
| Right pair echoes | D11 / D12 |

All modules require a common ground. Power the motors and servos from suitable
external supplies rather than USB. Disconnect both D0/D1 motor-board jumpers
while uploading firmware; otherwise the motor board and USB converter share
the only hardware UART.

The L293D profile uses A0-A3 for its four servos, A4/A5 for Bluetooth, one
shared ultrasonic trigger on D2, and one echo per direction on D9/D10/D13.
Consequently it supports distance assistance but not paired auto-alignment.

## Arm calibration gate

The committed configuration intentionally has `ArmCalibrated = false` in
`src/app/BuildConfig.h`. This prevents estimated geometry from enabling arm
presets or ranging automation.

1. Build and upload `arm_calibration` with the motor board disconnected.
2. Point each joint slowly through safe positions:

   ```sh
   PYTHONPATH=scripts python3 scripts/gamepad_motor.py \
     --port /dev/cu.usbmodemXXXX --calibrate-joint 0 --angle 90
   ```

3. Measure link lengths, servo zero positions/directions, joint limits,
   gripper offsets, cargo clearance, and safe preset poses.
4. Enter those values in `src/app/BuildConfig.h` and only then set
   `ArmCalibrated = true`.
5. First validate without cargo at low speed, then repeat with a lightweight
   test load.

The default preset policy changes only necessary dimensions when the direct
path satisfies the safety envelope. Unsafe or cargo-carrying stow operations
first lift to the configured clearance and then retract. The gripper never
automatically releases during link loss, emergency stop, or stowing.

## Python controller

Install the package:

```sh
python3 -m pip install -e .
```

List ports and inspect the Xbox mapping:

```sh
warehouse-robot --list-ports
warehouse-robot --monitor
```

Run the actual Bluetooth robot:

```sh
warehouse-robot --port /dev/cu.HC-05-DevB --baud 9600
```

Run the L293D development profile over USB or Bluetooth:

```sh
warehouse-robot --port /dev/cu.usbmodemXXXX --baud 115200 --legacy-ascii
warehouse-robot --port /dev/cu.usbmodemXXXX --baud 115200 \
  --send-command W:400,400,400,400 --duration 1
```

Saved Xbox mappings live in `scripts/robot_control/config.py`:

- Left stick: forward/back and rotation.
- LT/RT: left/right strafe.
- Right stick: arm rotation and extension.
- LB/RB: arm height down/up.
- A/B: open/close gripper.
- D-pad left/up/right: 0/90/180-degree presets; down: stow.
- Y: start ranging assist; X or a large stick movement: cancel it.
- Menu: latched emergency stop; View: clear emergency stop.

At 20 Hz, each binary frame atomically carries all axes and buttons, a sequence
number, protocol version, and CRC-8. The actual robot ignores malformed frames
and relies on the next frame rather than acknowledging over SoftwareSerial.

## Ultrasonic assistance

Sensor groups follow the arm direction: front near 90 degrees, left near 0,
and right near 180. One valid sensor provides range only. A pair first uses the
distance difference for low-speed alignment, then chooses reach using the
average distance and configured gripper offset.

The assist never approaches an object beyond arm reach. If the robot is too
close it may move slowly away. Invalid/stale readings, timeout, X, or large
manual input cancel automatic motion. Small operator corrections are blended
under the same low-speed limit. Ordinary manual driving does not receive a
global ultrasonic stop override.

## Verification

```sh
PYTHONPATH=scripts python3 -m unittest discover -s tests -v
python3 -m compileall -q scripts tests
pio test -e native
pio run -e l293d_dev -e uart_encoder_robot -e arm_calibration
```

Software reduces risk but cannot guarantee collision-free or drop-free motion
without verified calibration, physical limits, gripper feedback, and adequate
environment sensing. Keep a physical power cutoff accessible during hardware
bring-up.
