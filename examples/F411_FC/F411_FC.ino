// SPDX-License-Identifier: GPL-3.0-or-later
// Custom F411 FC: 8 MHz crystal, SPI2 PB13/PB14/PB15, ICM42605 CS PC13.
#include <Arduino.h>
#include <IMULibrary.h>
#include <ttimu/ArduinoBus.h>
#include <ttimu/ArduinoMonitor.h>
#include "FCConfig.h"

SPIClass sensorSPI(PB15, PB14, PB13); // MOSI, MISO, SCK; selects SPI2.
ttimu::ArduinoSPIBus sensorBus(sensorSPI, PC13);
ttimu::IMU imu;
ttimu::ArduinoMonitor<decltype(Serial)> monitor(imu, Serial);
bool initialized = false;

void setup() {
    Serial.begin(921600); // Native USB CDC on PA11/PA12, not a UART adapter.
    // MATEKF411TE shares SPI2 with MAX7456 OSD. Keep its CS inactive if populated.
    digitalWrite(PB12, HIGH);
    pinMode(PB12, OUTPUT);
    sensorBus.begin();
    sensorSPI.begin();
    ttimu::Config config;
    for (unsigned axis = 0; axis < 3; ++axis) config.axes[axis] = fc::Axes[axis];
    config.eulerRollSign = fc::EulerRollSign;
    initialized = imu.begin(sensorBus, ttimu::Sensor::ICM42605, config);
    if (initialized) monitor.begin();
}

void loop() {
    if (initialized) {
        monitor.update(); // Acquisition/fusion 1 kHz, CSV graph stream 100 Hz.
        return;
    }
    // Repeat init failures so a browser connected after boot can still see them.
    static uint32_t lastError = 0;
    if (Serial && millis() - lastError >= 1000) {
        lastError = millis();
        Serial.print("# ERROR: ");
        Serial.println(ttimu::IMU::errorName(imu.error()));
    }
}
