# Changelog

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
