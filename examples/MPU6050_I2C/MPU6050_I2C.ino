// SPDX-License-Identifier: GPL-3.0-or-later
#include <Arduino.h>
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>
#include <ttimu/ArduinoMonitor.h>
ttimu::ArduinoI2CBus sensorBus(Wire,0x68); // AD0 high -> 0x69
ttimu::IMU imu;
ttimu::ArduinoMonitor<decltype(Serial)> monitor(imu,Serial);
void setup(){
    Serial.begin(921600);
    // ESP32: Wire.begin(SDA_pin,SCL_pin). STM32: setSDA/setSCL before begin().
    Wire.begin();Wire.setClock(400000);
    if(!imu.begin(sensorBus,ttimu::Sensor::MPU6050)){Serial.print("# ERROR: ");Serial.println(ttimu::IMU::errorName(imu.error()));return;}
    monitor.begin();
}
void loop(){monitor.update();}
