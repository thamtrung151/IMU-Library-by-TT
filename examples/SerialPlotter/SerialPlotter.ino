// SPDX-License-Identifier: GPL-3.0-or-later
// ESP32-S3 + ICM42688 / ICM42605 / MPU6500. Edit pins for your board.
#include <Arduino.h>
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>
#include <ttimu/ArduinoMonitor.h>
constexpr int PIN_SCK=12, PIN_MISO=13, PIN_MOSI=11, PIN_CS=10;
ttimu::ArduinoSPIBus sensorBus(SPI,PIN_CS);
ttimu::IMU imu;
ttimu::ArduinoMonitor<decltype(Serial)> monitor(imu,Serial);
void setup(){
    Serial.begin(921600);SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI,PIN_CS);sensorBus.begin();
    if(!imu.begin(sensorBus)){Serial.print("# ERROR: ");Serial.println(ttimu::IMU::errorName(imu.error()));return;}
    monitor.begin();
}
void loop(){monitor.update();} // Sensor/fusion 1 kHz, graph output 100 Hz.
