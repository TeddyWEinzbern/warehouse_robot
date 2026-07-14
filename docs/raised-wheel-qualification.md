# Raised-wheel qualification and promotion checklist

Do not enable the normal UART profile merely because the firmware compiles.
`DriveCalibrationQualified` remains false until this checklist is completed and
the measured defaults are reviewed.

## Preconditions

- Robot is raised so all wheels rotate freely and cannot contact the floor.
- A physical motor-power cutoff is reachable by a second person.
- D0/D1, HC-05 voltage divider, common ground, and motor power are verified.
- `safe_idle` has been flashed first and produces no motor or servo attachment.
- HC-05 baud is confirmed; select the matching 38400 or 9600 build profile.
- Sonar and arm power/motion remain disabled during drivetrain qualification.

## Procedure

1. Flash `uart_closed_loop_qualification` (or its 9600 variant).
2. Capture motor UART traffic from reset. Verify the first complete frame is
   `$Car:0,0,0,0!`, zero repeats every 50 ms, motor type occurs after 100 ms,
   encoder polarity after 200 ms, and encoder qualification starts after 300 ms.
3. Verify three valid increment replies are required before `DISARMED` becomes
   ready for an arm request.
4. Clear the host E-stop, hold all controls neutral for at least 500 ms, then
   issue ARM from the local dashboard.
5. At no more than 200 mm/s, apply forward, reverse, left/right strafe, and
   left/right rotation one at a time. Record:
   - physical wheel driven by A/B/C/D;
   - required command sign per logical wheel;
   - encoder channel and sign per logical wheel;
   - measured speed versus `$Car:` target.
6. Measure wheel loaded diameter and verify counts per wheel revolution.
7. Vary query timing while observing `raw_increments` to determine whether
   `Encoder_20ms` is controller-fixed 20 ms data or counts accumulated between
   accepted samples.
8. Inject missing, truncated, malformed, mismatched, and stale replies. Verify
   immediate zero, exit/refusal of `ARMED`, latched fault, and recovery only
   after healthy feedback, neutral controls, explicit clear, and re-arm.
9. Hold a wheel, reverse direction, and introduce an encoder sign error. Verify
   qualification warnings identify stall/sign/scale candidates without adding
   an Arduino PID.
10. Add host jitter and telemetry load. Confirm `$Car:` remains on accumulated
    20 ms deadlines, scheduler counters stay below fault thresholds, and no
    catch-up bursts occur.
11. Validate low-profile acceleration/deceleration, 40 ms zero crossing, and
    immediate ramp bypass for E-stop/fault/link timeout.
12. Run for 30 minutes under representative serial and control load. Export the
    dashboard snapshots and record maximum loop gap, every missed-deadline
    counter, query timeouts, RX overflows, and telemetry drops.

## Promotion record

Record the reviewed results before editing defaults:

```text
Date / operators:
Motor-board firmware identity:
HC-05 baud:
$Car: numeric scale:
Command A/B/C/D logical map:
Command signs:
Encoder logical map:
Encoder signs:
Wheel diameter:
Counts per revolution:
Encoder_20ms semantics:
Low-profile test result:
Normal-profile test result:
30-minute test artifacts:
Physical cutoff verification:
```

Then update `RuntimeConfig::defaults()`, review the exact diff, and separately
approve `ROBOT_DRIVE_CALIBRATION_QUALIFIED=1`. Aggressive response remains
disabled until normal-profile driving validation passes.
