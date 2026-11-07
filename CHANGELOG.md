# Changelog

## 0.1.1 — 2026-11-07

- Fix public-header collisions with Arduino's `sq` and `radians` macros;
  Arduino.h can be included first without changing user macros or IMU math.
- Add a standalone PlatformIO ESP32-S3 robot project and prioritize PlatformIO
  installation in the README, including Git-tag and local-library workflows.
- Add a header compatibility regression test; all embedded examples now include
  Arduino.h first to exercise normal consumer include order.
- Add a custom F411 FC profile: 8 MHz HSE, 96 MHz CPU/48 MHz USB clock override,
  ICM42605 on SPI2/PC13, OSD chip deselection, native USB CSV and DFU upload.
- Add FC flashing/axis-check instructions and a Windows viewer launcher.
  Custom PCB orientation and hardware runtime remain unverified.

## 0.1.0 — 2026-11-03

First prototype of **IMU Library by TT**.

- Portable INAV 9-derived six-axis attitude core, startup gyro calibration,
  filters, gyro Kalman, diagnostics and quaternion/Euler/SI outputs.
- Drivers for ICM42605, ICM42688-P, MPU6500 and MPU6050.
- Arduino ESP32/STM32 SPI/I²C adapters and runnable examples.
- Offline Web Serial/CSV monitor with an explicitly labeled simulation mode.
- Host reference/behavior tests and ESP32-S3/STM32F411 compile checks.
- GPL license, pinned upstream snapshots and reproduction tools.

Hardware drift/temperature/vibration tests are pending. This release is a
prototype for evaluation, not a measured accuracy guarantee.
