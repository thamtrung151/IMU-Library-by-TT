// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "../IMULibrary.h"

namespace ttimu {
// Call SPI.begin(...) yourself first so board pins remain under your control.
class ArduinoSPIBus final : public RegisterBus {
public:
    ArduinoSPIBus(SPIClass& spi,uint32_t chipSelect,uint32_t hz=8000000)
        :spi_(spi),cs_(chipSelect),readHz_(hz) {}
    void begin() {pinMode(cs_,OUTPUT);digitalWrite(cs_,HIGH);}
    bool read(uint8_t reg,uint8_t* data,size_t size) override {
        const bool dataRegister=(reg>=0x1D&&reg<=0x30)||(reg>=0x3A&&reg<=0x48);
        spi_.beginTransaction(SPISettings(dataRegister?readHz_:1000000,MSBFIRST,SPI_MODE0));
        digitalWrite(cs_,LOW);spi_.transfer(reg|0x80);
        for(size_t i=0;i<size;++i)data[i]=spi_.transfer(0xFF);
        digitalWrite(cs_,HIGH);spi_.endTransaction();return true;
    }
    bool write(uint8_t reg,uint8_t value) override {
        // MPU register writes must stay at or below 1 MHz.
        spi_.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
        digitalWrite(cs_,LOW);spi_.transfer(reg&0x7F);spi_.transfer(value);
        digitalWrite(cs_,HIGH);spi_.endTransaction();return true;
    }
    void delayMs(uint32_t ms) override {delay(ms);}
    uint32_t micros32() override {return micros();}
    bool isSPI() const override {return true;}
private:
    SPIClass& spi_;uint32_t cs_,readHz_;
};

// Call Wire.begin()/set pins first. Supports MPU6050 and all other chips over I2C.
class ArduinoI2CBus final : public RegisterBus {
public:
    ArduinoI2CBus(TwoWire& wire,uint8_t address=0x68):wire_(wire),address_(address){}
    bool read(uint8_t reg,uint8_t* data,size_t size) override {
        wire_.beginTransmission(address_);wire_.write(reg);
        if(wire_.endTransmission(false)!=0)return false;
        const size_t received=wire_.requestFrom(address_,static_cast<uint8_t>(size));
        if(received!=size){while(wire_.available())wire_.read();return false;}
        for(size_t i=0;i<size;++i)data[i]=static_cast<uint8_t>(wire_.read());
        return true;
    }
    bool write(uint8_t reg,uint8_t value) override {
        wire_.beginTransmission(address_);wire_.write(reg);wire_.write(value);
        return wire_.endTransmission()==0;
    }
    void delayMs(uint32_t ms) override {delay(ms);}
    uint32_t micros32() override {return micros();}
    bool isSPI() const override {return false;}
private:
    TwoWire& wire_;uint8_t address_;
};
}
#endif
