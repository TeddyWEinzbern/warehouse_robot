# Sonar sensor configuration

How the ultrasonic (HC-SR04-style) distance sensors work in this firmware,
what hardware they need, how to run with anywhere from 0 to 6 of them, and
which files to edit when your wiring differs.

## 1. How the code works

Everything lives in `src/subsystems/SensorSubsystem.cpp`; all pin
assignments live in `src/app/PinProfile.h`.

1. Sensors are organized into **trigger groups**. All sensors in a group
   share one trigger pin and are fired together; groups take turns. With the
   default two groups, a group fires every 60 ms, so each individual sensor
   updates every 120 ms.
2. Firing a group = drive its trigger pin HIGH for 10 µs, then watch each
   member's echo pin. Echo-HIGH time is converted to millimetres
   (duration·10/58). A sensor whose echo never completes within 25 ms is
   stored as **invalid** — this is how unconnected sensors behave, and it is
   harmless.
3. Every sensor belongs to a **direction** (0 front, 1 left, 2 right) and a
   **slot** (0 or 1) within that direction. Readings land in a
   3-direction × 2-slot snapshot.
4. A per-sensor **offset** (±500 mm, from `sensorOffsetMm[6]`) is added to
   every reading before storage, so you can compensate for how deep each
   sensor sits behind the robot's outline. Offsets are runtime parameters
   (dashboard "sensor offset" fields or ParameterSet group 6); like all
   runtime parameters they reset on reboot, so put long-term values in
   `RuntimeConfig` via the dashboard only for testing.
5. Readings older than 300 ms are reported as stale/invalid in telemetry
   (`sensor_mm[6]` + validity mask in the sensor/arm telemetry frame). The
   assist subsystem uses front-direction readings for approach/alignment;
   it simply refuses to engage while the readings it needs are invalid.

## 2. Hardware configuration (default wiring)

| Sensor index | Direction | Group / trigger pin | Echo pin |
| --- | --- | --- | --- |
| 0 | front | 0 / D2 | D4 |
| 1 | left | 0 / D2 | D8 |
| 2 | right | 0 / D2 | D11 |
| 3 | front | 1 / D13 | D7 |
| 4 | left | 1 / D13 | D10 |
| 5 | right | 1 / D13 | D12 |

Requirements: 5 V supply and common ground for every sensor; one trigger
wire per **group** (shared), one echo wire per **sensor**. D13 is also the
Uno's LED pin — the group-1 trigger works, but confirm the wiring change
before relying on group 1.

## 3. How many sensors?

- **Minimum: 0.** Leave everything unconnected. All readings stay invalid;
  drive, arm, and telemetry work normally; assist features that need
  distance simply stay idle. No code change required.
- **Any subset (1–5):** wire only what you have, leave the rest
  unconnected. Unwired sensors read invalid and are ignored. Prefer filling
  group 0 (D2 trigger) first, and the front direction first — assist only
  uses front sensors.
- **Maximum: 6** — 3 directions × 2 slots. This is a structural limit: the
  snapshot, telemetry frame, and offset table are all sized for six.

## 4. Changing the configuration for your wiring

Edit the five arrays in `src/app/PinProfile.h` (UART branch). They must all
have one entry per sensor, in the same order:

```cpp
constexpr uint8_t SonarTrigger[]   = {2, 13};          // one pin per GROUP
constexpr uint8_t SonarEcho[]      = {4, 8, 11, 7, 10, 12}; // one pin per sensor
constexpr uint8_t SonarDirection[] = {0, 1, 2, 0, 1, 2};    // 0 front, 1 left, 2 right
constexpr uint8_t SonarSlot[]      = {0, 0, 0, 1, 1, 1};    // slot within the direction
constexpr uint8_t SonarGroup[]     = {0, 0, 0, 1, 1, 1};    // which trigger fires it
```

Typical changes:

- **Different pins:** change the number in `SonarEcho` (or `SonarTrigger`).
  Free pins on this robot are only the ones not listed in the wiring table
  of the [README](../README.md); a `static_assert` in the same file catches
  collisions with servo/host pins.
- **A sensor points a different way:** change its `SonarDirection` entry.
  Keep each (direction, slot) pair unique.
- **Only one trigger group:** shorten `SonarTrigger` to `{2}`, set every
  `SonarGroup` entry to 0, and give each direction at most two slots. All
  sensors then update every 60 ms instead of every 120 ms.
- **Fewer than six sensors, cleanly:** you may shorten all per-sensor
  arrays together (e.g. three front/left/right sensors on one group) — the
  array lengths are the single source of truth (`SonarCount` is derived).
  Leaving the arrays at six and not wiring the extras is equally fine.

After editing, rebuild and reflash the firmware profile you use
(`pio run -t upload -e robot_closed_loop` or `-e calibration`).

## 5. Filling in the values that describe your sensors

| Value | Where | Meaning |
| --- | --- | --- |
| Trigger/echo pins, direction, slot, group | `src/app/PinProfile.h` arrays above | Physical wiring and geometry roles |
| `sensorOffsetMm[6]` defaults (all 0) | set at runtime via dashboard/ParameterSet group 6, index 0–5 | +/− mm added to each reading so distances measure from the robot's outline, not the sensor face |
| `SensorStaleMs` (300), `SonarGroupPeriodUs` (60 ms) | `src/app/BuildConfig.h` | Staleness window and firing cadence — normally leave alone |

## 6. Verifying

1. Flash any profile (`calibration` or a robot build) and start the host:
   `warehouse-robot run --port ... --no-gamepad`, open the dashboard at
   `http://127.0.0.1:8765`.
2. The sensor panel shows six distances plus validity. Hold a large flat
   object ~30 cm in front of a sensor: its reading should sit near 300 mm
   and flag valid; unconnected sensors must show invalid, not garbage.
3. Measure the true distance with a ruler; if a sensor reads consistently
   long/short, set its offset (±500 mm) in the dashboard and re-check.
