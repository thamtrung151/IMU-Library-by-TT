// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
#include "ttimu/detail/InavCore.h"

namespace ttimu {
constexpr const char* Version = "0.1.2";
struct Vector3 { float x,y,z; Vector3(float a=0,float b=0,float c=0):x(a),y(b),z(c){} };
struct Quaternion { float w,x,y,z; Quaternion(float a=1,float b=0,float c=0,float d=0):w(a),x(b),y(c),z(d){} };
enum class Sensor : uint8_t { Auto, MPU6050, MPU6500, ICM42605, ICM42688 };
enum class Error : uint8_t { None, NotStarted, Bus, NotFound, WrongSensor, InvalidConfig, InvalidSample, TimingGap, CalibrationMismatch };
enum class State : uint8_t { Stopped, Calibrating, Ready, WaitingForCalibration, Error, WaitingForAttitude };

// Implement this interface to use STM32 HAL, ESP-IDF or any other platform.
// A single task must own the bus and IMU instance. No heap is used by the library.
class RegisterBus {
public:
    virtual ~RegisterBus() = default;
    virtual bool read(uint8_t reg, uint8_t* data, size_t size) = 0;
    virtual bool write(uint8_t reg, uint8_t value) = 0;
    virtual void delayMs(uint32_t ms) = 0;
    virtual uint32_t micros32() = 0;
    virtual bool isSPI() const = 0;
};

struct Config {
    // Robot profile: safe default for both SPI and 400 kHz I2C.
    uint16_t sampleRateHz = 1000;
    uint16_t fusionRateHz = 1000;
    uint16_t gyroAntiAliasHz = 250;
    uint16_t gyroLowpassHz = 60;
    uint16_t accelLowpassHz = 15;
    uint16_t gyroCalibrationMs = 2000;
    float gyroMovementThreshold = 32;
    bool gyroKalman = true;
    uint16_t kalmanQ = 100;
    bool startupCalibration = true;
    bool checkDataReady = true;
    uint32_t maxGapUs = 20000;
    // Signed, right-handed axis permutation: e.g. {2,-1,3} = rotated PCB.
    int8_t axes[3] = {1, 2, 3};
    static Config inav9(); // SPI ICM: sample 4 kHz, fusion 2 kHz.
};

struct RawSample {
    int16_t gyro[3] = {};
    int16_t accel[3] = {};
    float temperatureC = 0;
    uint32_t timestampUs = 0;
};

struct Calibration {
    uint32_t version = 1;
    Sensor sensor = Sensor::Auto;
    float gyroBiasCounts[3] = {};
    float accelOffsetG[3] = {};
    float accelScale[3] = {1,1,1};
};

struct Data {
    Quaternion quaternion; // Native INAV quaternion, Hamilton w,x,y,z.
    float roll = 0, pitch = 0, yaw = 0; // Degrees; Euler signs as in INAV MSP.
    Vector3 gyro;  // rad/s, calibrated and filtered body rates
    Vector3 accel; // m/s^2 including gravity, calibrated/filtered
    float temperatureC = 0;
    uint64_t timestampUs = 0; // elapsed since begin; extends micros() wrap
    uint32_t sequence = 0;
    bool ready = false;
    bool absoluteHeadingValid = false;
};

struct Diagnostics {
    uint32_t rawSamples = 0, updates = 0, readErrors = 0;
    uint32_t lateSamples = 0, timingGaps = 0, rejectedSamples = 0;
    uint32_t calibrationRetries = 0, quaternionRecoveries = 0;
    float calibrationProgress = 0, lastDtUs = 0, accelWeight = 0;
    Vector3 gyroDriftCorrection; // AHRS integral feedback in rad/s, separate from data().gyro.
    bool attitudeInitialized = false;
};

class IMU {
public:
    bool begin(RegisterBus& bus, Sensor expected = Sensor::Auto, const Config& config = Config());
    // Nonblocking: call often. True means a new output snapshot is available.
    bool update();
    // Portable raw replay / custom acquisition. Feed unique samples at configured rate.
    bool beginRaw(Sensor sensor, const Config& config = Config());
    bool feed(const RawSample& sample);
    void startCalibration();
    void zeroYaw();
    void setArmed(bool armed) { armed_ = armed; }
    void setRateSetpoint(const Vector3& degreesPerSecond);
    bool setCalibration(const Calibration& calibration);
    const Calibration& calibration() const { return calibration_; }
    const Data& data() const { return data_; }
    const Diagnostics& diagnostics() const { return diagnostics_; }
    Sensor sensor() const { return sensor_; }
    State state() const { return state_; }
    Error error() const { return error_; }
    bool ready() const { return data_.ready; }
    const Config& config() const { return config_; }
    static const char* sensorName(Sensor sensor);
    static const char* errorName(Error error);
private:
    bool configureDevice(Sensor expected);
    bool readDevice(RawSample& sample);
    bool initialize(Sensor sensor, const Config& config);
    bool calibrateGyro(const RawSample& sample, uint64_t elapsedUs);
    void resetFilters();
    void publish(const detail::fpVector3_t& gyro, const detail::fpVector3_t& acc, const RawSample& raw, uint64_t elapsedUs);
    RegisterBus* bus_ = nullptr;
    Sensor sensor_ = Sensor::Auto;
    State state_ = State::Stopped;
    Error error_ = Error::NotStarted;
    Config config_;
    Calibration calibration_;
    Data data_;
    Diagnostics diagnostics_;
    detail::InavCore core_;
    detail::pt1Filter_t antiAlias_[3] = {}, gyroLpf_[3] = {};
    detail::biquadFilter_t accelLpf_[3] = {};
    detail::kalman_t kalman_[3] = {};
    detail::stdev_t gyroVariance_[3] = {};
    float gyroSums_[3] = {}, lastGyro_[3] = {};
    uint32_t calibrationCount_ = 0, lastRawTimestamp_ = 0, lastPoll_ = 0;
    uint64_t elapsedUs_ = 0, lastFusionUs_ = 0, calibrationStartUs_ = 0;
    bool haveTimestamp_ = false, haveFusion_ = false, gyroCalibrated_ = false, armed_ = false, recoveredLastUpdate_ = false;
    bool attitudeInitialized_ = false;
};
}
