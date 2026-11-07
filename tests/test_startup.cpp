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
int main(int argc,char** argv){allFaces();recalibration();pendingGravityAndAxes();if(argc==2)trace(argv[1]);std::printf("PASS startup: %u checks\n",checks);}
