// SPDX-License-Identifier: GPL-3.0-or-later
#include <Arduino.h>
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>
#include <ttimu/ArduinoMonitor.h>

// User PCB: CS/SA0 high, I2C_SDA -> IO8, I2C_SCL -> IO9, INT1/2 unconnected.
constexpr uint8_t ImuAddress=0x6A;
constexpr int ImuSda=8,ImuScl=9;
ttimu::ArduinoI2CBus sensorBus(Wire,ImuAddress);
ttimu::IMU imu;
ttimu::ArduinoMonitor<decltype(Serial)> monitor(imu,Serial);

void setup(){
    Serial.begin(921600);
    Wire.begin(ImuSda,ImuScl);Wire.setClock(400000);
    if(!imu.begin(sensorBus,ttimu::Sensor::QMI8658B,ttimu::Config::qmi8658b())){
        Serial.print("# ERROR: ");Serial.println(ttimu::IMU::errorName(imu.error()));return;
    }
    monitor.begin();
}
void loop(){monitor.update();}
