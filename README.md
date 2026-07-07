# Arduino Uno Gamepad UART Motor Control

This project reads a gamepad on the computer, sends compact control commands to
an Arduino Uno over USB serial, then the Arduino sends the motor driver's UART
protocol from the provided `UART历程/驱动电机` example.

## Control Mapping

- Left stick Y: forward and backward
- Left stick X: rotate left and right
- Left trigger: strafe left
- Right trigger: strafe right

The computer sends commands to Arduino in this format:

```text
C:<forward>,<turn>,<strafe>
```

Each value is from `-1000` to `1000`. Arduino converts those values into the
four-wheel `$Car:a,b,c,d!` command used by the motor driver.

## Wiring Arduino Uno To The Motor Driver

The Uno uses USB serial for the computer gamepad bridge, so the motor driver is
connected to a separate software serial port:

- Arduino D11 (TX) -> motor driver RX
- Arduino D10 (RX) <- motor driver TX
- Arduino GND -> motor driver GND
- Motor driver power -> the battery or supply required by the driver and motors

Important notes:

- TX and RX are crossed.
- Arduino and the motor driver must share GND.
- Do not power motors from the Arduino 5V pin.
- If the motor driver UART is 3.3V-only, put a level shifter or divider between
  Arduino D11 and the driver's RX pin.
- The driver example uses `115200` baud. `SoftwareSerial` on Uno can be marginal
  at this speed, so if commands are unreliable, use a board with an extra
  hardware serial port such as Arduino Mega, or lower the driver baud rate if
  the driver supports it.

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

Stop with `Ctrl+C`. The bridge resends the current command every 100 ms, so a
held stick or trigger keeps moving the motors. When the Arduino stops receiving
commands for 300 ms, it automatically sends zero speed to the motor driver.

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
  --left-trigger-axis 2 \
  --right-trigger-axis 5
```

Some controllers report triggers as `-1..1`, while others report `0..1`. The
default `--trigger-mode auto` handles both common cases, but you can force it:

```sh
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX --trigger-mode signed
python3 scripts/gamepad_motor.py --port /dev/cu.usbmodemXXXX --trigger-mode unsigned
```
