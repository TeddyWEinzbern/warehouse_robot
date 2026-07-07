# Arduino Uno Gamepad L293D Motor Control

This project reads a gamepad on the computer, sends compact control commands to
an Arduino Uno over USB serial or an HC-05/HC-06 Bluetooth serial module, then
the Arduino directly drives four motors through L293D motor drivers.

## Control Mapping

- Left stick Y: forward and backward
- Left stick X: rotate left and right
- Left trigger: strafe left when `DriveMixing::Mecanum` is enabled
- Right trigger: strafe right when `DriveMixing::Mecanum` is enabled

The computer sends commands to Arduino in this format:

```text
C:<forward>,<turn>,<strafe>
```

Each value is from `-1000` to `1000`. Arduino converts those values into the
four-wheel PWM and direction outputs used by the L293D drivers.

For hardware diagnosis, the Arduino also accepts direct wheel commands:

```text
W:<front_left>,<front_right>,<rear_left>,<rear_right>
M:<motor_index>,<speed>
```

`M` motor indexes are `1=front_left`, `2=front_right`, `3=rear_left`,
`4=rear_right`.

The default Arduino mixing mode is `DriveMixing::Mecanum`. For mecanum wheels,
left stick Y drives forward/back, left stick X rotates, and LT/RT strafe. A
forward-only command such as `C:400,0,0` should drive all four wheels together.

```cpp
constexpr DriveMixing ACTIVE_DRIVE_MIXING = DriveMixing::Mecanum;
```

## Command Interface

The sketch can read commands from USB serial, Bluetooth serial, or both. Change
this constant in `src/main.cpp` when you want to switch:

```cpp
constexpr CommandInterface ACTIVE_COMMAND_INTERFACE = CommandInterface::Both;
```

Available values are `CommandInterface::Usb`, `CommandInterface::Bluetooth`, and
`CommandInterface::Both`.

## HC-05/HC-06 Bluetooth Wiring

- Arduino A5 (RX) <- HC-05/HC-06 TXD
- Arduino A4 (TX) -> HC-05/HC-06 RXD
- Arduino GND -> HC-05/HC-06 GND
- Arduino 5V -> HC-05/HC-06 VCC if your module board supports 5V input

Important notes:

- TX and RX are crossed.
- Many HC-05/HC-06 RXD pins are 3.3V logic, so use a voltage divider or level
  shifter from Arduino A4 to module RXD.
- The sketch uses `9600` baud for Bluetooth by default.

## L293D Motor Shield Wiring

This code targets the common L293D Motor Shield V1-compatible board shown in the
photo. Do not wire Arduino pins directly to the L293D chip pins; the shield uses
M1/M2/M3/M4 terminals plus an onboard shift register for direction control.

| Robot wheel | Shield terminal |
| --- | --- |
| Front left | M1 |
| Front right | M2 |
| Rear left | M3 |
| Rear right | M4 |

Important notes:

- The shield uses Arduino D12, D4, D7, and D8 for the shift-register direction
  latch.
- The shield uses Arduino D11, D3, D6, and D5 for M1, M2, M3, and M4 PWM speed.
- M3 and M4 use Uno PWM pins D6 and D5. Those pins run from Timer0 at a different
  PWM rate than M1/M2, and small DC motors often have different start thresholds.
  The sketch therefore applies a minimum moving PWM and a small rear-channel trim:

```cpp
constexpr uint8_t MAX_MOTOR_PWM = 180;
constexpr uint8_t FRONT_LEFT_MIN_PWM = 55;
constexpr uint8_t FRONT_RIGHT_MIN_PWM = 55;
constexpr uint8_t REAR_LEFT_MIN_PWM = 65;
constexpr uint8_t REAR_RIGHT_MIN_PWM = 65;
constexpr float REAR_LEFT_SPEED_SCALE = 1.08f;
constexpr float REAR_RIGHT_SPEED_SCALE = 1.08f;
```

  If M3/M4 still start later than M1/M2, raise only `REAR_LEFT_MIN_PWM` and
  `REAR_RIGHT_MIN_PWM` in small steps such as `5`. If the rear wheels become too
  fast after they are moving, lower only the rear speed scale values.
- Do not power motors from USB. Use the shield motor-power terminal or the
  shield's documented PWR jumper setup.
- Arduino logic power and motor power must share GND. On this shield, that is
  normally handled through the stacked headers and power terminal.
- If the shield has a PWR jumper, confirm whether it is intended to connect
  Arduino VIN to motor power. For a separate motor battery, follow the board's
  markings and avoid back-powering USB.
- If one motor spins the wrong direction, flip only that motor's polarity flag
  in `src/main.cpp`:

```cpp
constexpr bool FRONT_LEFT_REVERSE_POLARITY = false;
constexpr bool FRONT_RIGHT_REVERSE_POLARITY = false;
constexpr bool REAR_LEFT_REVERSE_POLARITY = false;
constexpr bool REAR_RIGHT_REVERSE_POLARITY = false;
```

- If every motor responds logically but the whole robot drives backward when the
  stick is pushed forward, flip `REVERSE_FORWARD_COMMAND`.
- If every motor responds logically but left/right turning is backwards, flip
  `REVERSE_TURN_COMMAND`.

## Build And Upload

Build:

```sh
pio run
```

Upload:

```sh
pio run -t upload
```

## Find The Arduino USB Port

On macOS, Arduino Uno usually appears as something like:

```sh
ls /dev/cu.usbmodem*
ls /dev/cu.usbserial*
```

The script can also list serial ports:

```sh
python3 scripts/gamepad_motor.py --list-ports
```

## Run The Gamepad Bridge

Install the Python dependencies:

```sh
python3 -m pip install pygame pyserial
```

Run the bridge:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX
```

For an HC-05/HC-06 Bluetooth serial port, use the Bluetooth port name and
`9600` baud:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600
```

Stop with `Ctrl+C`. The bridge resends the current command every 100 ms, so a
held stick or trigger keeps moving the motors. When the Arduino stops receiving
commands for 300 ms, it automatically stops all L293D outputs.

## Bluetooth Link Debugging

The Arduino prints one debug line per second on USB serial when
`DEBUG_SERIAL = true`:

```text
DBG usb bytes=0 ok=0 bad=0 | bt bytes=0 ok=0 bad=0 | last none C:0,0,0 wheels FL/FR/RL/RR=0/0/0/0 stopped=yes
```

To verify that the computer is actually reaching Arduino through HC-05/HC-06:

1. Keep USB connected and open the Arduino monitor:

```sh
pio device monitor -b 115200
```

2. In another terminal, send a fixed Bluetooth command:

```sh
python3 scripts/gamepad_motor.py \
  --port /dev/cu.HC-05-DevB \
  --baud 9600 \
  --send-command C:400,0,0 \
  --duration 1.5
```

If Bluetooth is working, `bt bytes` and `bt ok` should increase, and `last`
should become `Bluetooth C:400,0,0`. If `bt bytes` stays at `0`, Arduino is not
receiving bytes from the HC-05/HC-06 TX pin.

For mecanum forward testing, `C:400,0,0` should show:

```text
wheels FL/FR/RL/RR=400/400/400/400
```

If the debug line shows all four wheel targets but the robot does not move all
four motors, the problem is in the L293D enable/input/output wiring or motor
power path, not in the mixing math.

To bypass mecanum mixing completely, test raw outputs:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600 --send-command W:600,600,600,600 --duration 1.5
```

Then test each motor channel:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600 --send-command M:1,600 --duration 1
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600 --send-command M:2,600 --duration 1
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600 --send-command M:3,600 --duration 1
python3 scripts/gamepad_motor.py --port /dev/cu.HC-05-DevB --baud 9600 --send-command M:4,600 --duration 1
```

## Gamepad Axis Debugging

If the controls do not match your controller, first monitor the raw axes:

```sh
python3 scripts/gamepad_motor.py --monitor --rate 10
```

Move the left stick and both triggers, then map the changing axis numbers:

```sh
python3 scripts/gamepad_motor.py \
  --port /dev/cu.usbmodemXXXX \
  --left-x-axis 0 \
  --left-y-axis 1 \
  --left-trigger-axis 4 \
  --right-trigger-axis 5
```

The defaults match common Xbox SDL mappings on macOS: `left-x=0`, `left-y=1`,
`left-trigger=4`, `right-trigger=5`. Some Xbox/pygame combinations instead put
LT on axis `2`, so use `--monitor` once if LT still does not move.

Some controllers report triggers as `-1..1`, others report `0..1`, and some
report a centered axis where LT is negative and RT is positive. The default
`--trigger-mode auto` handles these common cases, but you can force it:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX --trigger-mode signed
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX --trigger-mode unsigned
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX --left-trigger-mode centered-negative --right-trigger-mode centered-positive
```
