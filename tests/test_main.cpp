// SPDX-License-Identifier: GPL-3.0-or-later
#include "IMULibrary.h"
#include "reference.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

static int assertions=0;
#define CHECK(x) do { ++assertions; if(!(x)){std::fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);std::exit(1);} }while(0)
using namespace ttimu;
using namespace ttimu::detail;
static float distance(const fpQuaternion_t& a,const fpQuaternion_t& b){
    float sum=0,sumNeg=0;
    const float av[]={a.q0,a.q1,a.q2,a.q3},bv[]={b.q0,b.q1,b.q2,b.q3};
    for(int i=0;i<4;++i){sum+=inavSquare(av[i]-bv[i]);sumNeg+=inavSquare(av[i]+bv[i]);}
    return sqrtf(std::min(sum,sumNeg));
}

struct FakeBus: RegisterBus {
    uint8_t bank=0;uint32_t clock=0;bool spi=true,failRead=false,failWrite=false;
    std::map<unsigned,uint8_t> registers;
    std::vector<std::pair<unsigned,unsigned>> writes;
    FakeBus(uint8_t id){registers[0x75]=id;}
    bool read(uint8_t r,uint8_t* out,size_t n) override{
        if(failRead)return false;
        for(size_t i=0;i<n;++i)out[i]=registers[(unsigned(bank)<<8)|unsigned(r+i)];return true;
    }
    bool write(uint8_t r,uint8_t v) override{
        if(failWrite)return false;writes.push_back({(unsigned(bank)<<8)|r,v});
        if(r==0x76)bank=v;else registers[(unsigned(bank)<<8)|r]=v;return true;
    }
    void delayMs(uint32_t ms) override{clock+=ms*1000;}
    uint32_t micros32() override{return clock;}
    bool isSPI() const override{return spi;}
    void put(uint8_t reg,int16_t v){registers[reg]=static_cast<uint16_t>(v)>>8;registers[reg+1]=static_cast<uint8_t>(v);}
};

static void referenceEquivalence(){
    InavCore port;port.reset();reference::reset();
    float maxError=0;
    for(uint32_t i=0;i<100000;++i){
        const float t=i*0.0005f;
        fpVector3_t gyro={{0.12f*sinf(t*1.7f),0.16f*cosf(t*0.9f),0.3f*sinf(t*0.7f)}};
        if(i%10000>8000)gyro.z=3.0f; // Rate and spin gates.
        fpVector3_t acc={{80*sinf(t),45*cosf(t*0.3f),Gravity+120*sinf(t*0.8f)}};
        if(i%7777<20)acc={{0,0,0}};
        const bool armed=i>10000 && i<70000;
        port.step(gyro,acc,0.0005f,i/2,armed);reference::step(gyro,acc,0.0005f,i/2,armed);
        const float error=distance(port.orientation,reference::quaternion());
        maxError=std::max(maxError,error);
        CHECK(error<2e-6f);CHECK(std::fabs(port.lastAccWeight-reference::weight())<1e-6f);
        const auto integral=reference::integral();
        for(unsigned a=0;a<3;++a)CHECK(std::fabs(port.vGyroDriftEstimate.v[a]-integral.v[a])<2e-7f);
    }
    std::printf("INAV full-function equivalence: 100000 steps, max quaternion distance %.9g\n",maxError);
}

static RawSample sample(uint32_t time){RawSample s;s.timestampUs=time;s.accel[2]=2048;s.temperatureC=25;return s;}
static void calibrationAndTiming(){
    IMU imu;CHECK(imu.beginRaw(Sensor::ICM42688));
    for(uint32_t i=0;i<2500;++i){auto s=sample(i*1000);s.gyro[0]=41;s.gyro[1]=-23;s.gyro[2]=17;CHECK(imu.feed(s));}
    CHECK(imu.ready());CHECK(std::fabs(imu.calibration().gyroBiasCounts[2]-17)<0.001f);
    CHECK(std::fabs(imu.data().gyro.z)<1e-6f);CHECK(std::fabs(imu.data().accel.z-9.80665f)<0.01f);
    CHECK(!imu.data().absoluteHeadingValid);
    auto repeated=sample(2499000);CHECK(!imu.feed(repeated));CHECK(imu.error()==Error::InvalidSample);
    auto gap=sample(2600000);CHECK(!imu.feed(gap));CHECK(!imu.ready());CHECK(imu.diagnostics().timingGaps==1);
    auto next=sample(2601000);CHECK(imu.feed(next));CHECK(imu.ready());
    CHECK(std::fabs(imu.data().yaw)<0.01f);
    // Every movement-window restart is visible; no early false completion.
    imu.startCalibration();
    for(uint32_t i=2602;i<4805;++i){auto s=sample(i*1000);s.gyro[0]=(i%2)?500:-500;imu.feed(s);}
    CHECK(!imu.ready());CHECK(imu.diagnostics().calibrationRetries>=1);
}

static void wrapAndInstances(){
    IMU a,b;Config cfg;cfg.gyroKalman=false;
    CHECK(a.beginRaw(Sensor::MPU6500,cfg));CHECK(b.beginRaw(Sensor::ICM42605,cfg));
    uint32_t start=0xFFFF0000u;
    for(uint32_t i=0;i<2205;++i){auto x=sample(start+i*1000);x.gyro[2]=33;a.feed(x);auto y=sample(i*1000);y.gyro[2]=-91;b.feed(y);}
    CHECK(a.ready()&&b.ready());CHECK(a.data().timestampUs==2204000);
    CHECK(std::fabs(a.calibration().gyroBiasCounts[2]-33)<0.01f);
    CHECK(std::fabs(b.calibration().gyroBiasCounts[2]+91)<0.01f);
    auto cal=a.calibration();cal.sensor=Sensor::ICM42688;CHECK(!a.setCalibration(cal));
    cal=a.calibration();cal.accelScale[0]=NAN;CHECK(!a.setCalibration(cal));
    Config invalid;invalid.axes[0]=-1;IMU bad;CHECK(!bad.beginRaw(Sensor::MPU6500,invalid));
    invalid=Config();invalid.fusionRateHz=777;CHECK(!bad.beginRaw(Sensor::MPU6500,invalid));
}

static void rotations(){
    InavCore core;core.reset();
    fpVector3_t gyro={{0,0,inavRadians(90)}},acc={{0,0,Gravity}};
    for(unsigned i=0;i<1000;++i)core.step(gyro,acc,0.001f,30000+i,true);
    float yaw=-atan2_approx(core.rMat[1][0],core.rMat[0][0])*180/M_PIf;
    CHECK(std::fabs(yaw+90)<0.02f);
    core.zeroYaw();CHECK(std::fabs(atan2_approx(core.rMat[1][0],core.rMat[0][0]))<1e-4f);
    const auto previous=core.orientation;core.integrate(0.001f,fpVector3_t{{NAN,0,0}},nullptr,0);
    CHECK(core.recoveries==1);CHECK(distance(previous,core.orientation)<1e-6f);
}

static void drivers(){
    for(uint8_t id:{uint8_t(0x42),uint8_t(0x47),uint8_t(0x70),uint8_t(0x68)}){
        FakeBus bus(id);if(id==0x68)bus.spi=false;IMU imu;CHECK(imu.begin(bus));
        const bool icm=id==0x42||id==0x47;
        if(icm){
            CHECK(bus.registers[0x4F]==6);CHECK(bus.registers[0x50]==6);CHECK((bus.registers[0x4D]&0xC0)==0x40);
            CHECK(bus.registers[0x10C]==(id==0x47?6:21));CHECK(bus.bank==0);
            bus.registers[0x2D]=8;bus.put(0x1D,0);bus.put(0x23,2048);
        }else{
            CHECK(bus.registers[0x1B]==0x18);CHECK(bus.registers[0x1C]==0x18);CHECK(bus.registers[0x19]==0);
            bus.registers[0x3A]=1;bus.put(0x3F,2048);bus.put(0x41,0);
        }
        bus.clock+=1000;CHECK(imu.update());CHECK(std::isfinite(imu.data().temperatureC));
        bus.failRead=true;bus.clock+=1000;CHECK(!imu.update());CHECK(imu.error()==Error::Bus);CHECK(!imu.ready());
    }
    FakeBus missing(0);IMU a;CHECK(!a.begin(missing));CHECK(a.error()==Error::NotFound);
    FakeBus wrong(0x47);CHECK(!a.begin(wrong,Sensor::MPU6050));CHECK(a.error()==Error::WrongSensor);
    FakeBus fast(0x47);Config cfg=Config::inav9();CHECK(a.begin(fast,Sensor::Auto,cfg));CHECK(fast.registers[0x4F]==4);
    FakeBus i2c(0x47);i2c.spi=false;CHECK(!a.begin(i2c,Sensor::Auto,cfg));
    FakeBus failed(0x47);failed.failWrite=true;CHECK(!a.begin(failed));
    FakeBus wrongBus(0x68);CHECK(!a.begin(wrongBus));CHECK(a.error()==Error::InvalidConfig);
    FakeBus floating(0x47);CHECK(a.begin(floating));floating.registers[0x2D]=0xFF;
    for(unsigned r=0x1D;r<0x1D+14;++r)floating.registers[r]=0xFF;
    floating.clock+=1000;CHECK(!a.update());CHECK(a.error()==Error::Bus);
}

int main(){referenceEquivalence();calibrationAndTiming();wrapAndInstances();rotations();drivers();
    std::printf("PASS: %d assertions; sizeof(IMU)=%zu bytes (host ABI). No hardware drift claim.\n",assertions,sizeof(IMU));return 0;}
