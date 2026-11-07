// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#if defined(ARDUINO)
#include <Arduino.h>
#include "../IMULibrary.h"

namespace ttimu {
// Shared CSV protocol for ESP32/STM32, SPI/I2C. Bounded TX; no String/heap allocation.
template<class SerialPort> class ArduinoMonitor {
    class Row : public Print {
    public:
        char bytes[512];size_t length=0;bool overflow=false;
        size_t write(uint8_t byte) override {
            if(length>=sizeof(bytes)){overflow=true;return 0;}
            bytes[length++]=static_cast<char>(byte);return 1;
        }
        void clear(){length=0;overflow=false;}
        void integer64(uint64_t value){
            char digits[21];unsigned n=0;
            do{digits[n++]=static_cast<char>('0'+value%10);value/=10;}while(value);
            while(n)write(digits[--n]);
        }
        void field(float value,uint8_t precision){write(',');print(value,precision);}
        void fieldUnsigned(uint32_t value){write(',');print(static_cast<unsigned long>(value));}
    } row_;
public:
    ArduinoMonitor(IMU& imu,SerialPort& serial,uint32_t outputPeriodUs=10000)
        :imu_(imu),serial_(serial),period_(outputPeriodUs){}
    void begin(){
        serial_.print("# IMU Library by TT v");serial_.print(Version);
        serial_.print(" / ");serial_.println(IMU::sensorName(imu_.sensor()));
        serial_.println("# Keep the IMU still for startup calibration (about 2 seconds).");
        serial_.println("t_us,roll,pitch,yaw,gx,gy,gz,ax,ay,az,temp,ready,dt_us,gaps,calib,retries,late,qw,qx,qy,qz,ix,iy,iz,acc_weight");
    }
    bool update(){
        if(offset_<row_.length){
            const int space=serial_.availableForWrite();
            if(space>0){size_t count=row_.length-offset_;if(count>static_cast<size_t>(space))count=space;
                offset_+=serial_.write(reinterpret_cast<const uint8_t*>(row_.bytes)+offset_,count);}
        }
        // Bound input work so a noisy host cannot monopolize sensor processing.
        for(unsigned n=0;n<32&&serial_.available();++n){
            char ch=static_cast<char>(serial_.read());
            if(ch=='\n'){
                command_[commandLength_]=0;
                if(!strcmp(command_,"CALIBRATE"))imu_.startCalibration();
                else if(!strcmp(command_,"ZERO_YAW"))imu_.zeroYaw();
                commandLength_=0;
            }else if(ch!='\r'&&commandLength_<sizeof(command_)-1)command_[commandLength_++]=ch;
        }
        const bool updated=imu_.update();
        const uint32_t now=micros();
        if(!updated||now-lastOutput_<period_||offset_<row_.length)return updated;
        lastOutput_=now;offset_=0;row_.clear();
        const Data& d=imu_.data();const Diagnostics& s=imu_.diagnostics();
        row_.integer64(d.timestampUs);
        row_.field(d.roll,3);row_.field(d.pitch,3);row_.field(d.yaw,3);
        row_.field(d.gyro.x,5);row_.field(d.gyro.y,5);row_.field(d.gyro.z,5);
        row_.field(d.accel.x,4);row_.field(d.accel.y,4);row_.field(d.accel.z,4);
        row_.field(d.temperatureC,2);row_.fieldUnsigned(d.ready?1:0);
        row_.field(s.lastDtUs,0);row_.fieldUnsigned(s.timingGaps);
        row_.field(s.calibrationProgress,3);row_.fieldUnsigned(s.calibrationRetries);row_.fieldUnsigned(s.lateSamples);
        row_.field(d.quaternion.w,6);row_.field(d.quaternion.x,6);row_.field(d.quaternion.y,6);row_.field(d.quaternion.z,6);
        row_.field(s.gyroDriftCorrection.x,7);row_.field(s.gyroDriftCorrection.y,7);row_.field(s.gyroDriftCorrection.z,7);
        row_.field(s.accelWeight,4);row_.write('\n');
        if(row_.overflow)row_.clear();
        return updated;
    }
private:
    IMU& imu_;SerialPort& serial_;uint32_t period_,lastOutput_=0;
    size_t offset_=0,commandLength_=0;char command_[24]={};
};
}
#endif
