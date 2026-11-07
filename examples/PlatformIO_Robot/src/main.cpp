// SPDX-License-Identifier: GPL-3.0-or-later
// ESP32-S3 robot + ICM42605/ICM42688/MPU6500 over SPI. Edit pins for your PCB.
#include <Arduino.h>
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>

constexpr int SCK_PIN = 12, MISO_PIN = 13, MOSI_PIN = 11, CS_PIN = 10;
ttimu::ArduinoSPIBus imuBus(SPI, CS_PIN);
ttimu::IMU imu;

void setup() {
    Serial.begin(921600);
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    imuBus.begin();

    if (!imu.begin(imuBus)) {
        Serial.println(ttimu::IMU::errorName(imu.error()));
        while (true) delay(100);
    }
}

void loop() {
    if (imu.update() && imu.ready()) {
        const auto& d = imu.data();
        // d.roll, d.pitch, d.yaw: degrees
        // d.gyro.x/y/z: radians/second; d.accel.x/y/z: m/s^2
        // Use this snapshot in your robot controller here.
        (void)d;
    }
    // Keep calling update() frequently. Avoid delay()/blocking motor or sensor I/O.
}
