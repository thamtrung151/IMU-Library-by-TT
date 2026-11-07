// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "InavMath.h"
#include "InavVectors.h"
namespace ttimu { namespace detail {
struct ImuRuntimeConfig {
    float dcm_kp_acc = 0.2f;
    float dcm_ki_acc = 0.005f;
    float dcm_kp_mag = 0.2f; // Also used by upstream's integral limit, even without MAG.
};
class InavCore {
public:
    fpQuaternion_t orientation = {1, 0, 0, 0};
    fpVector3_t vGyroDriftEstimate = {{0, 0, 0}};
    float rMat[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    ImuRuntimeConfig imuRuntimeConfig;
    float lastAccWeight = 0;
    uint32_t recoveries = 0;
    void reset();
    // Portable startup step: derive tilt from valid stationary gravity, yaw = 0.
    bool alignToAcceleration(const fpVector3_t& accCmss);
    void computeRotationMatrix();
    void step(const fpVector3_t& gyroRad, const fpVector3_t& accCmss,
              float dt, uint32_t uptimeMs, bool armed);
    void integrate(float dt, const fpVector3_t& gyroRad,
                   const fpVector3_t* accBF, float accWScaler);
    void zeroYaw();
private:
    pt1Filter_t rotationFilters_[3] = {};
};
}}
