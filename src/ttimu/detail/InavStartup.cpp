// SPDX-License-Identifier: GPL-3.0-or-later
// Portable startup policy, separate from the unchanged generated INAV update.
#include "InavCore.h"

namespace ttimu { namespace detail {
bool InavCore::alignToAcceleration(const fpVector3_t& accCmss) {
    const float normSq = vectorNormSquared(&accCmss);
    if (!std::isfinite(normSq) || normSq < inavSquare(0.8f * Gravity) ||
        normSq > inavSquare(1.2f * Gravity)) return false;
    // Used once after gyro calibration; valid for both upright and inverted starts.
    // R(q)^T * [0,0,1] must point along measured body-frame acceleration.
    const float roll = atan2f(accCmss.y, accCmss.z);
    const float pitch = atan2f(-accCmss.x, sqrtf(inavSquare(accCmss.y) + inavSquare(accCmss.z)));
    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    orientation = {cr * cp, sr * cp, cr * sp, -sr * sp};
    quaternionNormalize(&orientation, &orientation);
    vectorZero(&vGyroDriftEstimate);
    std::memset(rotationFilters_, 0, sizeof(rotationFilters_));
    lastAccWeight = 0;
    computeRotationMatrix();
    return true;
}
}}
