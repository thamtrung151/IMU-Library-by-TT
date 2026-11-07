# Port notes — v0.1.2

Upstream: INAV 9.0.1, `d44f2cf6e60cc9318b10551291b596370b5c957b`.
This is a **six-axis portable robot profile**, not full firmware equivalence.

## Preserved

- The no-heading accelerometer feedback, spin-rate integration gate (20 dps),
  body-axis integral clamp, Taylor/sine delta-quaternion integration and matrix
  equations are extracted from `flight/imu.c`.
- Kp_acc=0.2, Ki_acc=0.005, Kp_mag=0.2 retained in the clamp even with no MAG.
- Gaussian acc magnitude weighting (width 0.2), 3 Hz rate gate input,
  armed rate-ignore defaults (15±5 dps), startup gain×10 for 20 seconds disarmed.
- INAV PT1/biquad formulas, trig approximations and gyro Kalman computations.
- Gyro startup mean/stddev window and retry logic; calibration before alignment.
- Gyro scale `1/16.4`, acc scale 2048 LSB/g.

The source generator and vendored originals make the derivation reviewable.
The test reference compiles the **full upstream Mahony function**, with external
heading inputs absent, separately from the reduced production function. It shares
unchanged math primitives. It checks valid-data attitude math, not an entire FC's
driver/scheduler/default filter configuration or every recovery branch.

## Deliberate changes

| Change | Reason |
|---|---|
| Per-instance state, portable bus and explicit caller clock | Multiple sensors/MCUs, no flight-stack globals or RTOS requirement |
| Default sample/fusion 1 kHz/1 kHz | Practical starting rate for SPI and 400 kHz I²C; optional ICM SPI 4k/2k preset |
| No RPM, dynamic FFT notch, LULU, adaptive filter, MAG/GPS/airspeed/flight modes | Scope of this robot prototype; do not advertise byte equivalence with INAV defaults |
| Gyro Kalman has explicit optional rate setpoint input | No hidden PID dependency; default zero when used as a standalone sensor module |
| One coherent gyro+acc+temperature register burst and data-ready check | Avoid mismatched axes/read times and duplicate polling data |
| Big-endian ICM configuration, explicit FIFO bypass, reset/readback | Deterministic direct-register format, including after earlier firmware use |
| Correct nearest ICM AAF coefficients: 42688 258 Hz, 42605 249 Hz | Upstream `int8_t selectedFreq` can truncate Hz and select 303 Hz for 42688; this profile fixes it |
| Pause sensors during static AAF writes; disable AFSR | Preserve required register sequencing and the documented upstream workaround |
| MPU profile hardware LPF setting 1, output 500/1000 Hz | Bound acquisition for this initial robot driver; not native INAV MPU 4 kHz configuration |
| Reject bad timestamps, I/O, floating-high bus, invalid calibration/config; skip large gaps | Do not integrate one new measurement over an interval that was not observed |
| Finite input/state guard and previous-state recovery | Explicit correction of upstream validation weakness; no claim of identical invalid-input recovery |
| At least 100 samples in calibration window | Avoid completion from a few delayed samples; variance test still assumes physical stillness |
| Preserve settled acc filter state when gyro calibration completes | Prevent a false acceleration transient |
| Float Euler and 64-bit relative output timestamp | Retain precision rather than MSP's integer yaw; handle timer wrap |
| Prefix internal square/radians helpers | Avoid Arduino macro collisions without undefining user macros or changing formulas |
| Seed initial tilt from gravity after gyro calibration, including inverted starts | Hardware logs exposed false large rotations from an identity quaternion against negative Z; do not accumulate pre-calibration integral feedback |
| Ready requires gyro and initial tilt; 0.8–1.2 g initial magnitude guard | Avoid advertising a valid attitude before orientation can be initialized |

Startup policy lives in `InavStartup.cpp`, outside generated INAV math/update.
This is an explicit portable-profile change, not a claim that the original INAV
function includes this initializer. The reference test still compares the native
update math separately. See `reports/2026-11-07-stationary/README.md` for evidence.

The core is allocation-free. Filters cache coefficients; direct burst I/O is used.
The current Arduino adapter is synchronous SPI/I²C. DMA, hardware FIFO and an
IRQ-driven ESP-IDF worker are future optimizations, not features of this release.
There is no built-in gyro thermal-bias model or absolute heading reference.

Default accelerometer calibration is factory scale/zero offset, not INAV's
six-position workflow. Hardware noise, mounting, residual bias and temperature
still determine real accuracy. Physical validation is pending.
