// SPDX-License-Identifier: GPL-3.0-or-later
// Original register driver from QST QMI8658B Datasheet Rev D (QST-PD-B002-22).
// See docs/QMI8658B.md for the source and profile limitations.
#include "Qmi8658B.h"
#include "../../IMULibrary.h"

namespace ttimu { namespace detail {
namespace {
constexpr uint8_t Ctrl1=0x02,Ctrl2=0x03,Ctrl3=0x04,Ctrl5=0x06;
constexpr uint8_t Ctrl7=0x08,Ctrl8=0x09,Ctrl9=0x0A,Cal1L=0x0B;
constexpr uint8_t FifoCtrl=0x14,StatusInt=0x2D,TempL=0x33,Reset=0x60;
int16_t little16(const uint8_t* p){return static_cast<int16_t>((static_cast<uint16_t>(p[1])<<8)|p[0]);}

// Startup only. Runtime lock polling below never blocks or calls delayMs().
bool commandDone(RegisterBus& bus,bool expected){
    for(unsigned i=0;i<100;++i){
        uint8_t status=0;
        if(!bus.read(StatusInt,&status,1))return false;
        if(((status&0x80)!=0)==expected)return true;
        bus.delayMs(1);
    }
    return false;
}
bool readback(RegisterBus& bus,uint8_t reg,uint8_t expected){
    uint8_t value=0;return bus.read(reg,&value,1)&&value==expected;
}
}

bool qmi8658bConfigure(RegisterBus& bus,uint8_t odr){
    if(odr!=3&&odr!=4)return false;
    if(!bus.write(Reset,0xB0))return false;
    bus.delayMs(20); // Reset <=15 ms; verify before any sensor/CTRL9 operation.
    if(!readback(bus,0x4D,0x80)||!readback(bus,0x00,0x05))return false;
    // Four-wire SPI when used, auto-increment and little-endian output.
    if(!bus.write(Ctrl1,0x40)||!bus.write(Ctrl7,0)||!bus.write(FifoCtrl,0)||!bus.write(Ctrl8,0x80))return false;
    if(!bus.isSPI()){
        // Rev D section 11.2.1: I2C locking needs internal AHB clock gating off.
        if(!bus.write(Cal1L,1)||!bus.write(Ctrl9,0x12)||!commandDone(bus,true)||
           !bus.write(Ctrl9,0)||!commandDone(bus,false))return false;
    }
    const uint8_t accel=static_cast<uint8_t>(0x30|odr); // +/-16 g, 2048 LSB/g.
    const uint8_t gyro=static_cast<uint8_t>(0x70|odr);  // +/-2048 dps, 16 LSB/dps.
    // Keep default hardware bandwidth; existing software filters remain active.
    if(!bus.write(Ctrl2,accel)||!bus.write(Ctrl3,gyro)||!bus.write(Ctrl5,0)||!bus.write(Ctrl7,0x83))return false;
    bus.delayMs(160); // Gyro wakeup 150 ms plus >=3 samples of settling.
    return readback(bus,Ctrl1,0x40)&&readback(bus,Ctrl2,accel)&&readback(bus,Ctrl3,gyro)&&
        readback(bus,Ctrl5,0)&&readback(bus,Ctrl7,0x83)&&readback(bus,Ctrl8,0x80)&&readback(bus,FifoCtrl,0);
}

QmiReadResult qmi8658bRead(RegisterBus& bus,RawSample& sample){
    uint8_t status=0;
    // Reading Avail starts the lock. On I2C the transaction itself normally
    // exceeds 6/12 us; check once more to finish within this update when possible.
    if(!bus.read(StatusInt,&status,1))return QmiReadResult::BusError;
    if((status&3)==1&&!bus.read(StatusInt,&status,1))return QmiReadResult::BusError;
    if((status&3)!=3)return QmiReadResult::NoData;
    uint8_t bytes[14];
    // TEMP_L through GZ_H: one coherent snapshot, last byte releases the lock.
    if(!bus.read(TempL,bytes,sizeof(bytes)))return QmiReadResult::BusError;
    bool allHigh=true;for(uint8_t value:bytes)if(value!=0xFF)allHigh=false;
    if(allHigh)return QmiReadResult::BusError;
    sample.temperatureC=little16(bytes)/256.0f;
    for(unsigned i=0;i<3;++i){sample.accel[i]=little16(bytes+2+2*i);sample.gyro[i]=little16(bytes+8+2*i);}
    return QmiReadResult::Sample;
}
}}
