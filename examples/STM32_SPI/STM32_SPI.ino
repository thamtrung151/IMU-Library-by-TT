// SPDX-License-Identifier: GPL-3.0-or-later
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>
#include <ttimu/ArduinoMonitor.h>
SPIClass sensorSPI(PA7,PA6,PA5); // MOSI, MISO, SCK; edit for your board.
ttimu::ArduinoSPIBus sensorBus(sensorSPI,PA4);
ttimu::IMU imu;
ttimu::ArduinoMonitor<decltype(Serial)> monitor(imu,Serial);
void setup(){
    Serial.begin(921600);sensorSPI.begin();sensorBus.begin();
    if(!imu.begin(sensorBus)){Serial.print("# ERROR: ");Serial.println(ttimu::IMU::errorName(imu.error()));return;}
    monitor.begin();
}
void loop(){monitor.update();}
