# Source attribution

This project contains code adapted from **INAV 9.0.1**,
https://github.com/iNavFlight/inav, commit
`d44f2cf6e60cc9318b10551291b596370b5c957b`.

INAV, Cleanflight and Betaflight contributors hold copyright in the upstream
implementations. The original quaternion implementation credits **HaukeRa**,
with refactoring/adaptation by **DigitalEntity**. Upstream math comments also
credit the contributors and references retained in the original snapshots.
Original license notices and source are preserved under `vendor/inav-9.0.1/`.
That directory is source material; the library compiles only `src/`.

The adapted code and original project code in this repository are distributed
under **GNU GPL version 3 or, at your option, any later version**. See `LICENSE`.
Adaptations were made on **2026-11-03** for this initial portable release.
On **2026-11-07**, internal square/radians helper names were prefixed for Arduino
macro compatibility; their formulas and the vendored originals are unchanged.

| Upstream files | Portable implementation |
|---|---|
| `flight/imu.c` | `src/ttimu/detail/InavCore.cpp`: no-MAG/no-GPS feedback, integral clamp, delta-quaternion and matrix update |
| `common/vector.h`, `common/quaternion.h` | `src/ttimu/detail/InavVectors.h`: extracted vector/quaternion primitives |
| `common/maths.c` | `src/ttimu/detail/InavMath.cpp`: trig approximations, Gaussian, variance and numerical helpers |
| `common/filter.c` | `src/ttimu/detail/InavMath.cpp`: PT1 and biquad filters |
| `flight/kalman.c/.h` | `src/ttimu/detail/InavMath.cpp/.h`: gyro Kalman, per-instance state |
| `common/calibration.c`, `sensors/gyro.c` | `src/IMULibrary.cpp`: adapted startup window/variance/retry/offset flow |
| ICM42605/MPU6500 drivers, `fc/settings.yaml` | Register sequences, coefficients and defaults used by the portable drivers/profile |

`vendor/inav-9.0.1/manifest.json` records hashes of the original snapshots.
`tools/vendor_inav.py` reproduces the extracted code from the pinned checkout.
Safety guards, transport changes and profile differences are in `docs/PORTING.md`.
These modifications are not an official INAV release and are not endorsed by INAV.

Arduino cores, ESP-IDF, STM32 HAL and toolchains are **not bundled** in this
repository; they are separately installed dependencies with their own licenses.
When distributing a combined firmware, review its corresponding-source and
license obligations, including this GPL-derived library.
