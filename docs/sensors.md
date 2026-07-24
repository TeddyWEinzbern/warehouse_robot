# Sonar sensor configuration

How the ultrasonic (HC-SR04-style) distance sensors work in this firmware,
what hardware they need, and how to enable and calibrate up to six sensors.

When the flag is omitted, `ROBOT_SENSOR_ENABLED` defaults to `0`. The shipped
`calibration` environment sets it to `1` so sensors can be qualified. The
sensor subsystem and its fixed storage remain compiled and allocated in every
Uno image, but a disabled subsystem does not configure sensor pins, schedule
pings, or use readings.

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
   stored as **invalid**. An unconnected Uno input is electrically floating,
   however, so it can also look like a false pulse; never rely on a bare,
   unwired echo entry becoming invalid.
3. Every sensor belongs to a **direction** (0 front, 1 left, 2 right) and a
   **slot** (0 or 1) within that direction. Readings land in a
   3-direction × 2-slot snapshot.
4. Distances are measured from the sensor face. This implementation has no
   runtime or compiled distance-offset table.
5. Readings older than the compiled staleness limit are invalid. Consumers
   refuse sensor-assisted behavior when the readings they need are disabled,
   missing, or stale.

The normal `robot` protocol does not stream sonar arrays and has no remote
sensor-offset API. The `calibration` image returns one sensor report only in
response to an explicit calibration read.

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

- **Minimum: 0.** Leave `ROBOT_SENSOR_ENABLED=0` and leave the sensors
  unconnected. Drive and arm can operate without external distance sensors;
  sensor-dependent behavior stays unavailable.
- **Any subset (1–5):** shorten the four per-sensor arrays so they describe
  only installed sensors. Do not leave a configured echo pin floating.
  Prefer filling group 0 (D2 trigger) first, and the front direction first —
  assist only uses front sensors.
- **Maximum: 6** — 3 directions × 2 slots. This is a structural limit: the
  snapshot and calibration report are sized for six.

## 4. Changing the configuration for your wiring

Edit the five arrays in `src/app/PinProfile.h`. `SonarTrigger` has one entry
per trigger group. The other four arrays have one entry per sensor, in the
same sensor order:

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
  of the [README](../README.md). Compile-time checks reject collisions with
  D0/D1, HC-06, servo, other trigger, and other echo pins.
- **A sensor points a different way:** change its `SonarDirection` entry.
  Keep each (direction, slot) pair unique.
- **Only one trigger group:** shorten `SonarTrigger` to `{2}`, set every
  `SonarGroup` entry to 0, and give each direction at most two slots. All
  sensors then update every 60 ms instead of every 120 ms.
- **Fewer than six sensors, cleanly:** you may shorten all per-sensor
  arrays together (e.g. three front/left/right sensors on one group) — the
  array lengths are the single source of truth (`SonarCount` is derived).
  This is required unless unused echo inputs are held at a defined LOW level
  by verified external hardware; simply leaving them open is not supported.

After editing, set `ROBOT_SENSOR_ENABLED=1`, then rebuild and reflash the
firmware environment you use (`pio run -t upload -e robot` or
`-e calibration`). Do not enable the flag until pin collisions, voltage
levels, and all connected echoes have been checked.

## 5. Filling in the values that describe your sensors

| Value | Where | Meaning |
| --- | --- | --- |
| Trigger/echo pins, direction, slot, group | `src/app/PinProfile.h` arrays above | Physical wiring and geometry roles |
| `SensorStaleMs` (300), `SonarGroupPeriodUs` (60 ms) | `src/app/BuildConfig.h` | Staleness window and firing cadence — normally leave alone |

## 6. Verifying

1. Build and flash `calibration` with `ROBOT_SENSOR_ENABLED=1`, then start
   `warehouse-robot calibrate --port ... --baud 9600`.
2. Use the calibration sensor-read command shown by `help`. Each invocation
   requests one on-demand protocol-v3 report; there is no background sensor
   panel.
3. Hold a large flat object about 300 mm in front of one sensor. Its reading
   should be near 300 mm and valid. A configured-but-uninstalled echo is a
   wiring/configuration error, not an expected invalid reading.
4. Measure from the sensor face with a ruler. A consistent offset must be
   handled by physical placement or a reviewed code change; there is no
   offset parameter in protocol v3.
5. Repeat at several distances and for every installed sensor. Only after all
   readings and stale/invalid behavior are correct should the sensor flag be
   enabled in the normal `robot` image.
