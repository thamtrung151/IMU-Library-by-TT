// SPDX-License-Identifier: GPL-3.0-or-later
#include "IMULibrary.h"
#include "../examples/F411_FC/FCConfig.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <initializer_list>

using namespace ttimu;
using namespace ttimu::detail;
static unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::fprintf(stderr,"startup FAIL line %d: %s\n",__LINE__,#x); std::exit(1); } } while (0)

static void gravityMatches(const InavCore& core, const fpVector3_t& acc) {
    const float norm = sqrtf(vectorNormSquared(&acc));
    for (unsigned i=0;i<3;++i) CHECK(std::fabs(core.rMat[2][i]-acc.v[i]/norm)<2e-6f);
    CHECK(std::fabs(quaternionNormSqared(&core.orientation)-1)<2e-6f);
}
static float yaw(const InavCore& c) { return -atan2_approx(c.rMat[1][0],c.rMat[0][0])*180/M_PIf; }
static RawSample stationary(uint32_t time) {
    RawSample s;s.timestampUs=time;s.temperatureC=31.6f;
    s.accel[0]=55;s.accel[1]=-31;s.accel[2]=-2048; // Near the first hardware log.
    return s;
}
static void allFaces() {
    for (auto acc : {fpVector3_t{{0,0,Gravity}},fpVector3_t{{0,0,-Gravity}},
                    fpVector3_t{{Gravity,0,0}},fpVector3_t{{-Gravity,0,0}},
                    fpVector3_t{{0,Gravity,0}},fpVector3_t{{0,-Gravity,0}},
                    fpVector3_t{{26.6019f,-14.83731f,-981.31166f}}}) {
        InavCore core;core.reset();core.vGyroDriftEstimate={{0.001f,-0.002f,0.003f}};
        CHECK(core.alignToAcceleration(acc));gravityMatches(core,acc);
        CHECK(vectorNormSquared(&core.vGyroDriftEstimate)==0);
        const float previousYaw=yaw(core);
        for(unsigned i=0;i<10000;++i)core.step(fpVector3_t{{0,0,0}},acc,0.001f,150000+i,false);
        gravityMatches(core,acc);
        // Euler yaw is undefined at pitch +/-90; compare gravity there instead.
        if(std::fabs(acc.x)<Gravity*0.99f)CHECK(std::fabs(yaw(core)-previousYaw)<0.02f);
    }
    InavCore invalid;invalid.reset();
    for(auto acc : {fpVector3_t{{0,0,0}},fpVector3_t{{0,0,2*Gravity}},fpVector3_t{{NAN,0,Gravity}}}){
        CHECK(!invalid.alignToAcceleration(acc));CHECK(invalid.orientation.q0==1);
    }
}
static void recalibration() {
    IMU imu;CHECK(imu.beginRaw(Sensor::ICM42605));
    bool firstReady=false;
    for(uint32_t ms=0;ms<153000;++ms) {
        if(ms==150000) {imu.startCalibration();CHECK(!imu.ready());CHECK(!imu.diagnostics().attitudeInitialized);}
        imu.feed(stationary(ms*1000));
        if(imu.ready()) {
            if(!firstReady){CHECK(ms<=2010);firstReady=true;}
            CHECK(std::fabs(std::fabs(imu.data().roll)-180)<1.0f);
            CHECK(std::fabs(imu.data().pitch)<2);
            const float y=imu.data().yaw;CHECK(std::min(y,360-y)<0.02f);
            CHECK(std::fabs(imu.diagnostics().gyroDriftCorrection.z)<1e-6f);
        }
    }
    CHECK(firstReady&&imu.ready());CHECK(imu.diagnostics().attitudeInitialized);
}
static void pendingGravityAndAxes() {
    IMU imu;CHECK(imu.beginRaw(Sensor::ICM42605));
    for(uint32_t ms=0;ms<2200;++ms) {RawSample s;s.timestampUs=ms*1000;s.temperatureC=25;imu.feed(s);}
    CHECK(!imu.ready());CHECK(imu.state()==State::WaitingForAttitude);
    for(uint32_t ms=2200;ms<2300;++ms){auto s=stationary(ms*1000);s.accel[0]=s.accel[1]=0;s.accel[2]=2048;imu.feed(s);}
    CHECK(imu.ready());
    Config cfg;cfg.startupCalibration=false;cfg.gyroAntiAliasHz=cfg.gyroLowpassHz=cfg.accelLowpassHz=0;cfg.gyroKalman=false;
    for(unsigned a=0;a<3;++a)cfg.axes[a]=fc::Axes[a];
    IMU board;CHECK(board.beginRaw(Sensor::ICM42605,cfg));
    Calibration cal;cal.sensor=Sensor::ICM42605;CHECK(board.setCalibration(cal));CHECK(!board.ready());
    auto s=stationary(0);s.accel[0]=s.accel[1]=0;s.accel[2]=2048;s.gyro[0]=164;s.gyro[1]=328;s.gyro[2]=492;
    board.feed(s);CHECK(board.ready());
    CHECK(std::fabs(board.data().accel.z-9.80665f)<1e-5f);
    CHECK(std::fabs(board.data().gyro.x-inavRadians(-20))<1e-5f);
    CHECK(std::fabs(board.data().gyro.y-inavRadians(10))<1e-5f);
    CHECK(std::fabs(board.data().gyro.z-inavRadians(30))<1e-5f);
}
static void rollConventionAndContinuousRun() {
    Config native;native.startupCalibration=false;native.gyroKalman=false;
    native.gyroAntiAliasHz=native.gyroLowpassHz=native.accelLowpassHz=0;
    Config reversed=native;reversed.eulerRollSign=fc::EulerRollSign;
    CHECK(reversed.eulerRollSign==-1);
    IMU a,b;CHECK(a.beginRaw(Sensor::ICM42605,native));CHECK(b.beginRaw(Sensor::ICM42605,reversed));
    Calibration cal;cal.sensor=Sensor::ICM42605;CHECK(a.setCalibration(cal));CHECK(b.setCalibration(cal));
    RawSample s;s.temperatureC=25;s.accel[0]=-530;s.accel[1]=989;s.accel[2]=1713;s.gyro[0]=164;
    CHECK(a.feed(s));CHECK(b.feed(s));CHECK(a.ready()&&b.ready());
    CHECK(std::fabs(a.data().roll+b.data().roll)<1e-5f);
    CHECK(a.data().pitch==b.data().pitch&&a.data().yaw==b.data().yaw);
    CHECK(a.data().quaternion.w==b.data().quaternion.w&&a.data().quaternion.x==b.data().quaternion.x);
    CHECK(a.data().quaternion.y==b.data().quaternion.y&&a.data().quaternion.z==b.data().quaternion.z);
    CHECK(a.data().gyro.x==b.data().gyro.x);CHECK(a.rollRateRadS()==-b.rollRateRadS());
    Config invalid;invalid.eulerRollSign=0;IMU bad;CHECK(!bad.beginRaw(Sensor::ICM42605,invalid));

    // Production must not schedule calibration at 150 s. Cross that time while
    // rotating, and cross the micros() wrap as well. No manual calibration call.
    IMU continuous;CHECK(continuous.beginRaw(Sensor::ICM42605));
    for(uint32_t ms=0;ms<200000;++ms){
        RawSample input;input.timestampUs=0xFFFF0000u+ms*1000;input.temperatureC=25;input.accel[2]=2048;
        if(ms>=2500)input.gyro[2]=82; // 5 deg/s, after startup calibration.
        continuous.feed(input);
        if(ms>2100&&ms%1000==0)CHECK(continuous.ready());
    }
    CHECK(continuous.ready());CHECK(continuous.diagnostics().calibrationRetries==0);
    CHECK(continuous.calibration().gyroBiasCounts[2]==0);CHECK(continuous.data().timestampUs==199999000);
    CHECK(continuous.diagnostics().calibrationProgress==1);CHECK(continuous.state()==State::Ready);
}
static float headingDistance(float value) {return std::min(std::fabs(value),std::fabs(360-value));}
static void zeroReference() {
    // A zero pose must change output coordinates, never force the gravity AHRS flat.
    for(int sign : {-1,1}) {
        Config cfg;cfg.eulerRollSign=sign;cfg.startupCalibration=false;cfg.gyroKalman=false;
        cfg.gyroAntiAliasHz=cfg.gyroLowpassHz=cfg.accelLowpassHz=0;
        IMU native,tared;CHECK(native.beginRaw(Sensor::ICM42605,cfg));CHECK(tared.beginRaw(Sensor::ICM42605,cfg));
        CHECK(!tared.zeroAttitude());
        Calibration cal;cal.sensor=Sensor::ICM42605;CHECK(native.setCalibration(cal));CHECK(tared.setCalibration(cal));
        RawSample s;s.temperatureC=30;s.accel[0]=-530;s.accel[1]=989;s.accel[2]=1713;
        native.feed(s);tared.feed(s);CHECK(tared.ready());
        CHECK(std::fabs(tared.data().roll)>20);CHECK(std::fabs(tared.data().pitch)>10);
        CHECK(tared.zeroAttitude());CHECK(tared.data().referenceId==1);
        for(uint32_t ms=1;ms<=3000;++ms){
            s.timestampUs=ms*1000;native.feed(s);tared.feed(s);
            CHECK(std::fabs(tared.data().roll)<.01f&&std::fabs(tared.data().pitch)<.01f&&headingDistance(tared.data().yaw)<.01f);
            const auto a=native.nativeQuaternion(),b=tared.nativeQuaternion();
            CHECK(a.w==b.w&&a.x==b.x&&a.y==b.y&&a.z==b.z);
            CHECK(native.data().gyro.x==tared.data().gyro.x&&native.data().accel.z==tared.data().accel.z);
            CHECK(native.diagnostics().gyroDriftCorrection.x==tared.diagnostics().gyroDriftCorrection.x);
        }
        // Body-X rotation from a tilted zero pose must remain a body-X rotation.
        s.accel[0]=s.accel[1]=s.accel[2]=0;s.gyro[0]=1640;
        for(uint32_t ms=3001;ms<=3900;++ms){s.timestampUs=ms*1000;tared.feed(s);}
        CHECK(std::fabs(tared.data().roll-sign*90)<.03f);CHECK(std::fabs(tared.data().pitch)<.03f);
        CHECK(headingDistance(tared.data().yaw)<.03f);
        const auto q=tared.data().quaternion;CHECK(std::fabs(q.w-.70710678f)<.001f&&std::fabs(q.x-.70710678f)<.001f);
        s.gyro[0]=0;s.gyro[1]=492;
        for(uint32_t ms=3901;ms<=4200;++ms){s.timestampUs=ms*1000;tared.feed(s);}
        const float r=tared.data().roll,p=tared.data().pitch;const auto before=tared.nativeQuaternion();CHECK(headingDistance(tared.data().yaw)>1);
        tared.zeroYaw();CHECK(headingDistance(tared.data().yaw)<.01f);
        CHECK(std::fabs(tared.data().roll-r)<.01f&&std::fabs(tared.data().pitch-p)<.01f);
        CHECK(tared.nativeQuaternion().w==before.w&&tared.nativeQuaternion().z==before.z);
        CHECK(tared.zeroAttitude());CHECK(tared.data().referenceId==3);
        tared.startCalibration(true);CHECK(!tared.ready());CHECK(!tared.zeroAttitude());
        s.accel[0]=700;s.accel[1]=-1000;s.accel[2]=1646;s.gyro[0]=41;s.gyro[1]=-23;s.gyro[2]=17;
        bool first=false;
        for(uint32_t ms=4201;ms<15000;++ms){s.timestampUs=ms*1000;tared.feed(s);if(tared.ready()){
            first=true;CHECK(std::fabs(tared.data().roll)<.01f&&std::fabs(tared.data().pitch)<.01f&&headingDistance(tared.data().yaw)<.01f);
        }}
        CHECK(first&&tared.ready());CHECK(tared.data().referenceId==5);
        CHECK(std::fabs(tared.calibration().gyroBiasCounts[0]-41)<.001f);
        CHECK(std::fabs(tared.nativeQuaternion().x)>.1f); // Tilt remains in the fusion state.
        CHECK(tared.beginRaw(Sensor::ICM42605,cfg));CHECK(tared.data().referenceId==0);
    }
    // A pending zero waits through calibration AND invalid/freefall acceleration.
    IMU waiting;CHECK(waiting.beginRaw(Sensor::ICM42605));waiting.startCalibration(true);
    RawSample s;s.temperatureC=25;
    for(uint32_t ms=0;ms<2200;++ms){s.timestampUs=ms*1000;waiting.feed(s);}
    CHECK(waiting.state()==State::WaitingForAttitude);CHECK(!waiting.ready());
    s.accel[1]=2048;
    for(uint32_t ms=2200;ms<2350;++ms){s.timestampUs=ms*1000;waiting.feed(s);if(waiting.ready()){
        CHECK(std::fabs(waiting.data().roll)<.01f&&std::fabs(waiting.data().pitch)<.01f);
    }}
    CHECK(waiting.ready());CHECK(waiting.data().referenceId==1);
}
static void trace(const char* path) {
    FILE* f=std::fopen(path,"w");CHECK(f);
    std::fprintf(f,"t,old_roll,old_pitch,old_yaw,new_roll,new_pitch,new_yaw,old_iz,new_iz\n");
    const float r=inavRadians(-3.418f),p=inavRadians(-6.091f),h=inavRadians(-359.813f);
    InavCore old,fixed;old.reset();fixed.reset();
    const float cr=cosf(r/2),sr=sinf(r/2),cp=cosf(p/2),sp=sinf(p/2),ch=cosf(h/2),sh=sinf(h/2);
    old.orientation={cr*cp*ch+sr*sp*sh,sr*cp*ch-cr*sp*sh,cr*sp*ch+sr*cp*sh,cr*cp*sh-sr*sp*ch};old.computeRotationMatrix();
    const fpVector3_t a={{26.6019f,-14.83731f,-981.31166f}},corrected={{a.y,a.x,-a.z}},g={{0,0,0}};
    CHECK(fixed.alignToAcceleration(corrected));
    for(unsigned i=0;i<30000;++i){
        old.step(g,a,0.001f,134647+i,false);fixed.step(g,corrected,0.001f,134647+i,false);
        if(i%10==0)std::fprintf(f,"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f\n",i*.001f,
            atan2_approx(old.rMat[2][1],old.rMat[2][2])*180/M_PIf,asinf(constrainf(-old.rMat[2][0],-1,1))*180/M_PIf,yaw(old),
            atan2_approx(fixed.rMat[2][1],fixed.rMat[2][2])*180/M_PIf,asinf(constrainf(-fixed.rMat[2][0],-1,1))*180/M_PIf,yaw(fixed),old.vGyroDriftEstimate.z,fixed.vGyroDriftEstimate.z);
    }
    std::fclose(f);
}
int main(int argc,char** argv){allFaces();recalibration();pendingGravityAndAxes();rollConventionAndContinuousRun();zeroReference();if(argc==2)trace(argv[1]);std::printf("PASS startup: %u checks\n",checks);}
