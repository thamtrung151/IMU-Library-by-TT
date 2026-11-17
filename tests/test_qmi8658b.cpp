// SPDX-License-Identifier: GPL-3.0-or-later
#include "IMULibrary.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

using namespace ttimu;
static unsigned checks=0;
#define CHECK(x) do{++checks;if(!(x)){std::fprintf(stderr,"QMI FAIL line %d: %s\n",__LINE__,#x);std::exit(1);}}while(0)
static bool near(float a,float b,float tolerance=1e-5f){return std::fabs(a-b)<tolerance;}

// Models documented sensor side effects, including CTRL9 handshake and the
// delayed STATUSINT lock. A burst consumes a sample only at its last GZ_H byte.
struct QmiBus final : RegisterBus {
    std::array<uint8_t,128> regs{};
    std::vector<std::pair<uint8_t,size_t>> reads;
    std::vector<std::pair<uint8_t,uint8_t>> writes;
    uint32_t clock=0,lockStart=0;
    bool spi=false,available=false,locking=false,commandDone=false;
    bool failAllReads=false,failLegacyProbe=true,badReset=false,neverDone=false,neverAck=false;
    int failRead=-1,failWrite=-1,corruptRead=-1;
    unsigned bursts=0,runtimeDelays=0,readLatencyUs=0;
    bool running=false;
    QmiBus(){reset();}
    void reset(){regs.fill(0);regs[0]=5;regs[1]=0x7C;regs[0x4D]=badReset?0:0x80;available=locking=commandDone=false;}
    bool read(uint8_t reg,uint8_t* out,size_t size) override {
        struct Advance {uint32_t& time;unsigned elapsed;~Advance(){time+=elapsed;}} advance{clock,readLatencyUs};
        reads.push_back({reg,size});
        if(failAllReads||reg==failRead||(reg==0x75&&failLegacyProbe))return false;
        if(reg==0x2D&&size==1){
            uint8_t status=commandDone?0x80:0;
            if(available){if(!locking){locking=true;lockStart=clock;}
                status|=1;if(clock-lockStart>=(regs[3]%16==4?12u:6u))status|=2;}
            *out=status;return true;
        }
        for(size_t i=0;i<size;++i)out[i]=regs.at(reg+i)^(reg==corruptRead?1:0);
        if(reg==0x33&&size==14){CHECK(available&&locking);++bursts;available=locking=false;}
        return true;
    }
    bool write(uint8_t reg,uint8_t value) override {
        writes.push_back({reg,value});if(reg==failWrite)return false;
        if(reg==0x60&&value==0xB0){reset();return true;}
        regs.at(reg)=value;
        if(reg==0x0A){if(value==0x12)commandDone=!neverDone;else if(value==0&&!neverAck)commandDone=false;}
        return true;
    }
    void delayMs(uint32_t ms) override{clock+=ms*1000;if(running)++runtimeDelays;}
    uint32_t micros32() override{return clock;}
    bool isSPI() const override{return spi;}
    void put(uint8_t reg,int16_t value){regs[reg]=static_cast<uint8_t>(value);regs[reg+1]=static_cast<uint16_t>(value)>>8;}
    void queue(){put(0x33,-1408);put(0x35,-1024);put(0x37,512);put(0x39,2048);
        put(0x3B,160);put(0x3D,-320);put(0x3F,480);available=true;locking=false;}
};
static Config direct(){Config c=Config::qmi8658b();c.startupCalibration=false;c.gyroKalman=false;
    c.gyroAntiAliasHz=c.gyroLowpassHz=c.accelLowpassHz=0;return c;}
static void loadBias(IMU& imu){Calibration c;c.sensor=Sensor::QMI8658B;CHECK(imu.setCalibration(c));}
static void initAndUnits(){
    CHECK(static_cast<unsigned>(Sensor::ICM42688)==4);CHECK(static_cast<unsigned>(Sensor::QMI8658B)==5);
    for(bool spi : {false,true}){
        QmiBus bus;bus.spi=spi;IMU imu;CHECK(imu.begin(bus,Sensor::QMI8658B,direct()));
        CHECK(std::strcmp(IMU::sensorName(imu.sensor()),"QMI8658B")==0);
        CHECK(bus.regs[2]==0x40&&bus.regs[3]==0x33&&bus.regs[4]==0x73);
        CHECK(bus.regs[6]==0&&bus.regs[8]==0x83&&bus.regs[0x14]==0);
        CHECK(near(imu.sampleRateHz(),896.8f,.001f));CHECK(near(imu.fusionRateHz(),896.8f,.001f));
        CHECK(imu.config().sampleRateHz==1000);CHECK(!imu.ready());loadBias(imu);
        bus.running=true;bus.clock+=1116;bus.queue();
        CHECK(!imu.update());CHECK(bus.bursts==0);CHECK(imu.diagnostics().rawSamples==0);
        CHECK(!imu.update());CHECK(bus.bursts==0); // No unbounded lock wait or delay.
        bus.clock+=12;CHECK(imu.update());CHECK(imu.ready());CHECK(bus.bursts==1);
        const Data& d=imu.data();
        CHECK(near(d.temperatureC,-5.5f));CHECK(near(d.accel.x,-4.903325f));
        CHECK(near(d.accel.y,2.4516625f)&&near(d.accel.z,9.80665f));
        CHECK(near(d.gyro.x,10*3.14159265358979323846f/180));
        CHECK(near(d.gyro.y,-20*3.14159265358979323846f/180));
        CHECK(near(d.gyro.z,30*3.14159265358979323846f/180));
        CHECK(near(imu.diagnostics().lastDtUs,1000000.0f/896.8f,.01f));
        bus.clock+=1116;CHECK(!imu.update());CHECK(imu.diagnostics().rawSamples==1);
        bus.queue();CHECK(!imu.update());bus.clock+=12;bus.failRead=0x33;
        CHECK(!imu.update());CHECK(imu.error()==Error::Bus&&!imu.ready());
        CHECK(imu.diagnostics().readErrors==1);bus.failRead=-1;
        CHECK(imu.update());CHECK(imu.ready());CHECK(bus.runtimeDelays==0);
    }
}
static void identificationAndFaults(){
    QmiBus bus;IMU imu;CHECK(imu.begin(bus));CHECK(imu.sensor()==Sensor::QMI8658B);
    CHECK(!imu.begin(bus,Sensor::MPU6050));CHECK(imu.error()==Error::WrongSensor);
    QmiBus absent;absent.regs[0]=0;CHECK(!imu.begin(absent));CHECK(imu.error()==Error::NotFound);
    QmiBus failed;failed.failAllReads=true;CHECK(!imu.begin(failed));CHECK(imu.error()==Error::Bus);
    for(int reg : {0x60,0x02,0x03,0x04,0x06,0x08,0x09,0x0A,0x0B,0x14}){
        QmiBus b;b.failWrite=reg;CHECK(!imu.begin(b,Sensor::QMI8658B));CHECK(!imu.ready());CHECK(imu.error()==Error::Bus);
    }
    for(int reg : {0x4D,0x02,0x03,0x04,0x06,0x08,0x14}){
        QmiBus b;b.corruptRead=reg;CHECK(!imu.begin(b,Sensor::QMI8658B));CHECK(imu.error()==Error::Bus);
    }
    for(unsigned stuck=0;stuck<3;++stuck){QmiBus b;b.badReset=stuck==0;b.neverDone=stuck==1;b.neverAck=stuck==2;
        CHECK(!imu.begin(b,Sensor::QMI8658B));CHECK(b.clock<500000);CHECK(!imu.ready());}
    QmiBus b;CHECK(imu.begin(b,Sensor::QMI8658B,direct()));loadBias(imu);
    b.clock+=1116;b.queue();CHECK(!imu.update());b.clock+=12;
    for(unsigned reg=0x33;reg<=0x40;++reg)b.regs[reg]=0xFF;
    CHECK(!imu.update());CHECK(imu.error()==Error::Bus&&!imu.ready());
}
static void ratesAndMotion(){
    QmiBus b;IMU imu;Config c=Config::qmi8658b();c.sampleRateHz=c.fusionRateHz=500;
    CHECK(imu.begin(b,Sensor::QMI8658B,c));CHECK(b.regs[3]==0x34&&b.regs[4]==0x74);
    CHECK(near(imu.sampleRateHz(),448.4f,.001f));
    for(unsigned invalid=0;invalid<4;++invalid){c=Config::qmi8658b();
        if(invalid==0)c=Config::inav9();
        if(invalid==1){c.sampleRateHz=c.fusionRateHz=500;c.gyroAntiAliasHz=225;}
        if(invalid==2)c.fusionRateHz=100;
        if(invalid==3)c.gyroAntiAliasHz=449;
        CHECK(!imu.beginRaw(Sensor::QMI8658B,c));CHECK(imu.error()==Error::InvalidConfig);
    }
    c=direct();CHECK(imu.beginRaw(Sensor::QMI8658B,c));loadBias(imu);
    RawSample s;s.temperatureC=25;s.accel[2]=2048;CHECK(imu.feed(s));
    for(unsigned i=1;i<=896;++i){s.timestampUs=static_cast<uint32_t>(std::lround(i*1000000.0/896.8));s.gyro[2]=1440;CHECK(imu.feed(s));}
    const float expected=360-90*s.timestampUs/1000000.0f;
    CHECK(near(imu.data().yaw,expected,.03f));CHECK(imu.diagnostics().lateSamples==0);
    CHECK(imu.zeroAttitude());CHECK(imu.data().yaw<.001f);CHECK(imu.data().referenceId==1);
    c.fusionRateHz=500;CHECK(imu.beginRaw(Sensor::QMI8658B,c));loadBias(imu);
    s.gyro[2]=0;s.timestampUs=0;CHECK(imu.feed(s));
    s.timestampUs=1116;CHECK(!imu.feed(s));s.timestampUs=2231;CHECK(imu.feed(s));
    CHECK(near(imu.fusionRateHz(),448.4f,.001f));CHECK(near(imu.diagnostics().lastDtUs,2231,.01f));
    // Native QMI counts must also be used for calibration and the stored bias.
    c=Config::qmi8658b();CHECK(imu.beginRaw(Sensor::QMI8658B,c));imu.startCalibration(true);
    s.gyro[0]=16;s.gyro[1]=-32;s.gyro[2]=8;s.accel[0]=700;s.accel[1]=-1000;s.accel[2]=1646;
    for(unsigned i=0;i<2500;++i){s.timestampUs=i*1116;imu.feed(s);}
    CHECK(imu.ready());CHECK(near(imu.calibration().gyroBiasCounts[0],16));
    CHECK(near(imu.data().gyro.x,0)&&near(imu.data().gyro.y,0)&&near(imu.data().gyro.z,0));
    CHECK(std::fabs(imu.data().roll)<.02f&&std::fabs(imu.data().pitch)<.02f);
    auto saved=imu.calibration();saved.sensor=Sensor::ICM42605;CHECK(!imu.setCalibration(saved));
    // The coherent-read protocol remains mandatory when the legacy flag is off.
    c=direct();c.checkDataReady=false;CHECK(imu.begin(b,Sensor::QMI8658B,c));loadBias(imu);
    b.clock+=1116;b.queue();CHECK(!imu.update());b.clock+=12;CHECK(imu.update());
    // A realistic I2C status transaction already lasts longer than the lock.
    // Complete both status reads and the burst within one update when possible.
    QmiBus slow;slow.readLatencyUs=90;CHECK(imu.begin(slow,Sensor::QMI8658B,direct()));loadBias(imu);
    slow.clock+=1116;slow.queue();CHECK(imu.update());CHECK(slow.bursts==1);
    CHECK(near(imu.data().temperatureC,-5.5f));
    // Physical PT1 step response must use QMI's effective ODR, not 1000/500 Hz.
    for(unsigned rate : {1000u,500u}){
        c=direct();c.sampleRateHz=c.fusionRateHz=rate;c.gyroLowpassHz=60;
        CHECK(imu.beginRaw(Sensor::QMI8658B,c));loadBias(imu);
        RawSample step;step.temperatureC=25;step.accel[2]=2048;CHECK(imu.feed(step));
        step.timestampUs=rate==1000?1116:2231;step.gyro[0]=160;CHECK(imu.feed(step));
        const float odr=rate==1000?896.8f:448.4f;
        const float k=(2*3.14159265358979323846f*60)/odr;
        CHECK(near(imu.data().gyro.x,(10*3.14159265358979323846f/180)*k/(1+k),1e-6f));
    }
}
int main(){initAndUnits();identificationAndFaults();ratesAndMotion();std::printf("PASS QMI8658B: %u checks; modeled register I/O, no physical hardware test\n",checks);}
