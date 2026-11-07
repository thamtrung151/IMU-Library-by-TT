// SPDX-License-Identifier: GPL-3.0-or-later
// INAV-derived processing: see THIRD_PARTY_NOTICES.md and docs/PORTING.md.
#include "IMULibrary.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace ttimu {
using namespace detail;
namespace {
int16_t be16(const uint8_t* p) { return static_cast<int16_t>((static_cast<uint16_t>(p[0])<<8)|p[1]); }
bool validAxes(const int8_t* a) {
    unsigned used = 0; int sign = 1, permutation[3];
    for (unsigned i=0;i<3;++i) {
        int v=a[i]; if (!v || v < -3 || v > 3) return false;
        if (v<0) { sign=-sign; v=-v; }
        if (used & (1u<<v)) return false;
        used |= 1u<<v; permutation[i]=v;
    }
    for (unsigned i=0;i<3;++i) for(unsigned j=i+1;j<3;++j) if(permutation[i]>permutation[j]) sign=-sign;
    return sign==1;
}
bool validConfig(const Config& c, Sensor s, bool spi) {
    if (!c.sampleRateHz || !c.fusionRateHz || c.sampleRateHz % c.fusionRateHz ||
        c.fusionRateHz>c.sampleRateHz || c.fusionRateHz<100 || c.sampleRateHz>8000 ||
        c.gyroCalibrationMs<500 || c.gyroCalibrationMs>30000 ||
        !std::isfinite(c.gyroMovementThreshold) || c.gyroMovementThreshold<=0 ||
        c.maxGapUs<2000 || c.maxGapUs>100000 || !validAxes(c.axes) || (c.gyroKalman && c.kalmanQ==0)) return false;
    if (c.gyroAntiAliasHz>=c.sampleRateHz/2 || c.gyroLowpassHz>=c.fusionRateHz/2 || c.accelLowpassHz>=c.fusionRateHz/2) return false;
    if (!spi && c.sampleRateHz>1000) return false;
    if (s==Sensor::MPU6050 || s==Sensor::MPU6500) return c.sampleRateHz==500 || c.sampleRateHz==1000;
    return c.sampleRateHz==500 || c.sampleRateHz==1000 || c.sampleRateHz==2000 || c.sampleRateHz==4000 || c.sampleRateHz==8000;
}
}

Config Config::inav9() { Config c; c.sampleRateHz=4000; c.fusionRateHz=2000; return c; }

bool IMU::initialize(Sensor s,const Config& c) {
    if (s==Sensor::Auto || !validConfig(c,s,bus_ ? bus_->isSPI() : true)) {
        state_=State::Error; error_=Error::InvalidConfig; data_.ready=false; return false;
    }
    sensor_=s; config_=c; calibration_=Calibration(); calibration_.sensor=s;
    data_=Data(); diagnostics_=Diagnostics(); core_.reset();
    elapsedUs_=lastFusionUs_=calibrationStartUs_=0;
    lastRawTimestamp_=lastPoll_=0; haveTimestamp_=haveFusion_=gyroCalibrated_=attitudeInitialized_=false; armed_=false;
    state_=State::Calibrating; resetFilters(); startCalibration();
    if(!c.startupCalibration) state_=State::WaitingForCalibration;
    error_=Error::None; return true;
}

void IMU::resetFilters() {
    std::memset(antiAlias_,0,sizeof(antiAlias_));
    std::memset(gyroLpf_,0,sizeof(gyroLpf_));
    std::memset(accelLpf_,0,sizeof(accelLpf_));
    for(unsigned a=0;a<3;++a) {
        if(config_.gyroAntiAliasHz) pt1FilterInit(&antiAlias_[a],config_.gyroAntiAliasHz,1.0f/config_.sampleRateHz);
        if(config_.gyroLowpassHz) pt1FilterInit(&gyroLpf_[a],config_.gyroLowpassHz,1.0f/config_.fusionRateHz);
        if(config_.accelLowpassHz) biquadFilterInit(&accelLpf_[a],config_.accelLowpassHz,1000000u/config_.fusionRateHz,1.0f/sqrtf(2.0f),FILTER_LPF);
        gyroKalmanInitAxis(&kalman_[a],config_.kalmanQ);
        lastGyro_[a]=0;
    }
}

bool IMU::beginRaw(Sensor s,const Config& c) { bus_=nullptr; return initialize(s,c); }

bool IMU::begin(RegisterBus& bus,Sensor expected,const Config& c) {
    bus_=&bus; config_=c; error_=Error::None; state_=State::Stopped; data_.ready=false;
    uint8_t id=0;
    if(!bus.read(0x75,&id,1)) {error_=Error::Bus;state_=State::Error;return false;}
    Sensor detected=Sensor::Auto;
    if(id==0x42) detected=Sensor::ICM42605;
    else if(id==0x47) detected=Sensor::ICM42688;
    else if(id==0x70) detected=Sensor::MPU6500;
    else if(id==0x68 || id==0x69) detected=Sensor::MPU6050;
    if(detected==Sensor::Auto) {error_=Error::NotFound;state_=State::Error;return false;}
    if(expected!=Sensor::Auto && expected!=detected) {error_=Error::WrongSensor;state_=State::Error;return false;}
    if(detected==Sensor::MPU6050 && bus.isSPI()) {error_=Error::InvalidConfig;state_=State::Error;return false;}
    if(!initialize(detected,c)) return false;
    if(!configureDevice(expected)) {error_=Error::Bus;state_=State::Error;data_.ready=false;return false;}
    lastPoll_=bus.micros32();
    return true;
}

bool IMU::configureDevice(Sensor) {
    auto write=[this](uint8_t r,uint8_t v,uint32_t delay=0) {bool ok=bus_->write(r,v);if(delay)bus_->delayMs(delay);return ok;};
    uint8_t value=0;
    if(sensor_==Sensor::ICM42605 || sensor_==Sensor::ICM42688) {
        // INAV register sequence; corrected nearest AAF selection (no int8 Hz truncation).
        if(!write(0x76,0) || !write(0x11,0x01,2) || !write(0x76,0) || !write(0x4C,0x30) || !write(0x4E,0x0F,15)) return false;
        uint8_t odr=6;
        switch(config_.sampleRateHz) {case 500:odr=15;break;case 2000:odr=5;break;case 4000:odr=4;break;case 8000:odr=3;break;default:break;}
        if(!write(0x4F,odr,15) || !write(0x50,odr,15) || !write(0x52,0xFF,15)) return false;
        const uint8_t delt=sensor_==Sensor::ICM42688 ? 6 : 21;
        const uint16_t deltSq=sensor_==Sensor::ICM42688 ? 36 : 440;
        const uint8_t shift=sensor_==Sensor::ICM42688 ? 10 : 6;
        // Stop both sensors while updating static filter registers (TDK requirement).
        if(!write(0x4E,0,1) || !write(0x76,1) || !write(0x0C,delt) || !write(0x0D,deltSq&255) || !write(0x0E,static_cast<uint8_t>((deltSq>>8)|(shift<<4)))) return false;
        if(!write(0x76,2) || !write(0x03,static_cast<uint8_t>(delt<<1)) || !write(0x04,deltSq&255) || !write(0x05,static_cast<uint8_t>((deltSq>>8)|(shift<<4)))) return false;
        if(!write(0x76,0) || !write(0x16,0) || !write(0x14,0x03) || !write(0x63,0,100) || !write(0x65,0x08)) return false;
        if(!bus_->read(0x64,&value,1) || !write(0x64,static_cast<uint8_t>((value&~0x10)|0x60),15)) return false;
        if(!bus_->read(0x4D,&value,1) || !write(0x4D,static_cast<uint8_t>((value&~0xC0)|0x40),15) || !write(0x4E,0x0F,45)) return false;
        uint8_t gyro=0,acc=0;
        return bus_->read(0x4F,&gyro,1) && bus_->read(0x50,&acc,1) && gyro==odr && acc==odr;
    }
    // Robot MPU profile: 1 kHz internal rate + hardware DLPF, no FIFO.
    if(!write(0x6B,0x80,100) || !write(0x6B,0x03,15) || !write(0x6C,0) || !write(0x1B,0x18) || !write(0x1C,0x18)) return false;
    if(!write(0x1A,1) || !write(0x19,static_cast<uint8_t>(1000/config_.sampleRateHz-1))) return false;
    if(sensor_==Sensor::MPU6500 && !write(0x1D,1)) return false;
    if(!write(0x23,0) || !write(0x6A,bus_->isSPI()?0x10:0) || !write(0x37,0) || !write(0x38,1,50)) return false;
    uint8_t gyro=0,acc=0;
    return bus_->read(0x1B,&gyro,1) && bus_->read(0x1C,&acc,1) && gyro==0x18 && acc==0x18;
}

bool IMU::readDevice(RawSample& sample) {
    const bool icm=sensor_==Sensor::ICM42605 || sensor_==Sensor::ICM42688;
    uint8_t bytes[14],status=0;
    if(config_.checkDataReady) {
        if(!bus_->read(icm?0x2D:0x3A,&status,1)) {error_=Error::Bus;++diagnostics_.readErrors;data_.ready=false;return false;}
        if(!(status&(icm?8:1))) return false;
    }
    if(!bus_->read(icm?0x1D:0x3B,bytes,sizeof(bytes))) {error_=Error::Bus;++diagnostics_.readErrors;data_.ready=false;return false;}
    bool allHigh=true;
    for(uint8_t byte:bytes)if(byte!=0xFF)allHigh=false;
    if(allHigh){error_=Error::Bus;++diagnostics_.readErrors;data_.ready=false;return false;}
    int16_t temperature;
    if(icm) {
        temperature=be16(bytes);
        for(unsigned a=0;a<3;++a) {sample.accel[a]=be16(bytes+2+a*2);sample.gyro[a]=be16(bytes+8+a*2);}
        sample.temperatureC=temperature/132.48f+25;
    } else {
        for(unsigned a=0;a<3;++a) {sample.accel[a]=be16(bytes+a*2);sample.gyro[a]=be16(bytes+8+a*2);}
        temperature=be16(bytes+6);
        sample.temperatureC=sensor_==Sensor::MPU6050 ? temperature/340.0f+36.53f : temperature/333.87f+21.0f;
    }
    return true;
}

bool IMU::update() {
    if(!bus_ || state_==State::Stopped || state_==State::Error) return false;
    const uint32_t now=bus_->micros32();
    const uint32_t period=1000000u/config_.sampleRateHz;
    if(static_cast<uint32_t>(now-lastPoll_)<period) return false;
    RawSample raw; raw.timestampUs=now;
    if(!readDevice(raw)) {
        if(haveTimestamp_ && static_cast<uint32_t>(now-lastRawTimestamp_)>config_.maxGapUs) {data_.ready=false;if(error_!=Error::Bus)error_=Error::TimingGap;}
        return false;
    }
    lastPoll_=now;
    return feed(raw);
}

void IMU::startCalibration() {
    if(sensor_==Sensor::Auto || state_==State::Error)return;
    calibrationCount_=0; calibrationStartUs_=elapsedUs_; gyroCalibrated_=false;
    for(unsigned a=0;a<3;++a) {gyroSums_[a]=0;devClear(&gyroVariance_[a]);calibration_.gyroBiasCounts[a]=0;}
    state_=State::Calibrating; data_.ready=false; diagnostics_.calibrationProgress=0;
    core_.reset(); resetFilters(); haveFusion_=false; attitudeInitialized_=false;
    diagnostics_.attitudeInitialized=false;diagnostics_.gyroDriftCorrection=Vector3();
}

bool IMU::calibrateGyro(const RawSample& sample,uint64_t now) {
    if(gyroCalibrated_ || state_==State::WaitingForCalibration) return false;
    for(unsigned a=0;a<3;++a) {gyroSums_[a]+=sample.gyro[a];devPush(&gyroVariance_[a],sample.gyro[a]);}
    ++calibrationCount_;
    const uint64_t window=static_cast<uint64_t>(config_.gyroCalibrationMs)*1000;
    diagnostics_.calibrationProgress=std::min(1.0f,static_cast<float>(now-calibrationStartUs_)/static_cast<float>(window));
    if(now-calibrationStartUs_<=window || calibrationCount_<100) return false;
    bool moved=false;
    for(unsigned a=0;a<3;++a) if(devStandardDeviation(&gyroVariance_[a])>config_.gyroMovementThreshold)moved=true;
    if(moved) {
        ++diagnostics_.calibrationRetries; calibrationStartUs_=now; calibrationCount_=0;
        for(unsigned a=0;a<3;++a) {gyroSums_[a]=0;devClear(&gyroVariance_[a]);}
        diagnostics_.calibrationProgress=0; return false;
    }
    for(unsigned a=0;a<3;++a) calibration_.gyroBiasCounts[a]=gyroSums_[a]/calibrationCount_;
    gyroCalibrated_=true; state_=State::Ready;
    return true;
}

bool IMU::feed(const RawSample& raw) {
    if(state_==State::Stopped || state_==State::Error)return false;
    if(!std::isfinite(raw.temperatureC)) {error_=Error::InvalidSample;data_.ready=false;++diagnostics_.rejectedSamples;return false;}
    uint32_t delta=1000000u/config_.sampleRateHz;
    if(haveTimestamp_) {
        delta=raw.timestampUs-lastRawTimestamp_;
        if(!delta || delta>0x7FFFFFFFu) {error_=Error::InvalidSample;data_.ready=false;++diagnostics_.rejectedSamples;return false;}
        elapsedUs_+=delta;
    } else {haveTimestamp_=true;calibrationStartUs_=0;}
    lastRawTimestamp_=raw.timestampUs; ++diagnostics_.rawSamples;
    error_=Error::None;
    const uint32_t samplePeriod=1000000u/config_.sampleRateHz;
    if(delta>samplePeriod+samplePeriod/2) diagnostics_.lateSamples+=delta/samplePeriod-1;
    if(delta>config_.maxGapUs) {
        ++diagnostics_.timingGaps; data_.ready=false; error_=Error::TimingGap;
        haveFusion_=false; resetFilters();
        if(!gyroCalibrated_) {calibrationCount_=0;calibrationStartUs_=elapsedUs_;for(unsigned a=0;a<3;++a){gyroSums_[a]=0;devClear(&gyroVariance_[a]);}}
        return false; // Never integrate one current sample across an unobserved interval.
    }
    const bool finished=calibrateGyro(raw,elapsedUs_);
    if(finished) {
        // Bias changes affect only gyro. Preserve the settled accelerometer state.
        for(unsigned a=0;a<3;++a) {
            antiAlias_[a].state=0; gyroLpf_[a].state=0;
            const float setpoint=kalman_[a].setpoint;
            gyroKalmanInitAxis(&kalman_[a],config_.kalmanQ);kalman_[a].setpoint=setpoint;
        }
    }
    float sensorGyro[3],sensorAcc[3];
    for(unsigned a=0;a<3;++a) {
        sensorGyro[a]=gyroCalibrated_?(raw.gyro[a]-calibration_.gyroBiasCounts[a])*(1.0f/16.4f):0;
        sensorAcc[a]=(raw.accel[a]/2048.0f-calibration_.accelOffsetG[a])*calibration_.accelScale[a];
    }
    fpVector3_t gyro,acc;
    for(unsigned a=0;a<3;++a) {
        const int axis=config_.axes[a];const unsigned index=static_cast<unsigned>((axis<0?-axis:axis)-1);
        const float g=axis<0?-sensorGyro[index]:sensorGyro[index];
        lastGyro_[a]=config_.gyroAntiAliasHz?pt1FilterApply(&antiAlias_[a],g):g;
        gyro.v[a]=lastGyro_[a];
        acc.v[a]=(axis<0?-sensorAcc[index]:sensorAcc[index]);
    }
    const uint32_t fusionPeriod=1000000u/config_.fusionRateHz;
    if(haveFusion_ && elapsedUs_-lastFusionUs_<fusionPeriod) return false;
    const float dt=haveFusion_?static_cast<float>(elapsedUs_-lastFusionUs_)*1e-6f:1.0f/config_.fusionRateHz;
    lastFusionUs_=elapsedUs_;haveFusion_=true;
    for(unsigned a=0;a<3;++a) {
        float g=config_.gyroLowpassHz?pt1FilterApply(&gyroLpf_[a],gyro.v[a]):gyro.v[a];
        if(config_.gyroKalman) {updateAxisVariance(&kalman_[a],g);g=kalman_process(&kalman_[a],g);}
        gyro.v[a]=inavRadians(g);
        if(config_.accelLowpassHz) acc.v[a]=biquadFilterApply(&accelLpf_[a],acc.v[a]);
        acc.v[a]*=Gravity;
    }
    const uint32_t previousRecoveries=core_.recoveries;
    // Do not build AHRS integral error against an invented level pose during gyro calibration.
    // Seed tilt from the settled accelerometer once the stationary gyro window has passed.
    if(gyroCalibrated_ && !attitudeInitialized_) {
        attitudeInitialized_=core_.alignToAcceleration(acc);
        state_=attitudeInitialized_?State::Ready:State::WaitingForAttitude;
    }
    if(attitudeInitialized_) core_.step(gyro,acc,dt,static_cast<uint32_t>(elapsedUs_/1000),armed_);
    recoveredLastUpdate_=core_.recoveries!=previousRecoveries;
    if(recoveredLastUpdate_)error_=Error::InvalidSample;
    diagnostics_.lastDtUs=dt*1e6f; diagnostics_.accelWeight=core_.lastAccWeight;
    diagnostics_.quaternionRecoveries=core_.recoveries; ++diagnostics_.updates;
    diagnostics_.attitudeInitialized=attitudeInitialized_;
    diagnostics_.gyroDriftCorrection=Vector3(core_.vGyroDriftEstimate.x,core_.vGyroDriftEstimate.y,core_.vGyroDriftEstimate.z);
    publish(gyro,acc,raw,elapsedUs_);
    return true;
}

void IMU::publish(const fpVector3_t& g,const fpVector3_t& a,const RawSample& raw,uint64_t time) {
    data_.quaternion={core_.orientation.q0,core_.orientation.q1,core_.orientation.q2,core_.orientation.q3};
    data_.roll=atan2_approx(core_.rMat[2][1],core_.rMat[2][2])*(180/M_PIf);
    data_.pitch=(0.5f*M_PIf-acos_approx(constrainf(-core_.rMat[2][0],-1,1)))*(180/M_PIf);
    data_.yaw=-atan2_approx(core_.rMat[1][0],core_.rMat[0][0])*(180/M_PIf);
    if(data_.yaw<0)data_.yaw+=360;
    data_.gyro.x=g.x;data_.gyro.y=g.y;data_.gyro.z=g.z;
    data_.accel.x=a.x*0.01f;data_.accel.y=a.y*0.01f;data_.accel.z=a.z*0.01f;
    data_.temperatureC=raw.temperatureC;data_.timestampUs=time;data_.sequence=diagnostics_.updates;
    data_.ready=gyroCalibrated_ && attitudeInitialized_ && std::isfinite(data_.yaw) && !recoveredLastUpdate_;
    data_.absoluteHeadingValid=false;
}

void IMU::zeroYaw() {if(!attitudeInitialized_)return;core_.zeroYaw();data_.yaw=0;data_.quaternion={core_.orientation.q0,core_.orientation.q1,core_.orientation.q2,core_.orientation.q3};}
void IMU::setRateSetpoint(const Vector3& v) {
    if(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)) {kalman_[0].setpoint=v.x;kalman_[1].setpoint=v.y;kalman_[2].setpoint=v.z;}
}
bool IMU::setCalibration(const Calibration& c) {
    if(c.version!=1 || c.sensor!=sensor_) {error_=Error::CalibrationMismatch;return false;}
    for(unsigned a=0;a<3;++a) if(!std::isfinite(c.gyroBiasCounts[a]) || std::fabs(c.gyroBiasCounts[a])>32767 ||
        !std::isfinite(c.accelOffsetG[a]) || std::fabs(c.accelOffsetG[a])>2 || !std::isfinite(c.accelScale[a]) || c.accelScale[a]<0.5f || c.accelScale[a]>2) {error_=Error::CalibrationMismatch;return false;}
    calibration_=c;
    if(!config_.startupCalibration) {gyroCalibrated_=true;state_=attitudeInitialized_?State::Ready:State::WaitingForAttitude;diagnostics_.calibrationProgress=1;}
    error_=Error::None;return true;
}
const char* IMU::sensorName(Sensor s) {
    switch(s){case Sensor::MPU6050:return "MPU6050";case Sensor::MPU6500:return "MPU6500";case Sensor::ICM42605:return "ICM42605";case Sensor::ICM42688:return "ICM42688-P";default:return "Auto/unknown";}
}
const char* IMU::errorName(Error e) {
    switch(e){case Error::None:return "OK";case Error::NotStarted:return "Not started";case Error::Bus:return "Bus read/write failed";case Error::NotFound:return "IMU not detected";case Error::WrongSensor:return "Unexpected IMU model";case Error::InvalidConfig:return "Unsupported rate/filter/axis configuration";case Error::InvalidSample:return "Invalid timestamp or sample";case Error::TimingGap:return "Sampling gap; angle during gap unavailable";case Error::CalibrationMismatch:return "Invalid calibration or wrong sensor";default:return "Unknown error";}
}
}
