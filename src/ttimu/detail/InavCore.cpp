// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from INAV 9.0.1, d44f2cf6e60cc9318b10551291b596370b5c957b; see THIRD_PARTY_NOTICES.md.
#include "InavCore.h"
#define DEGREES_TO_RADIANS(x) ((x) * (M_PIf / 180.0f))
#define SPIN_RATE_LIMIT 20
namespace ttimu { namespace detail {
void InavCore::reset() {
    orientation = {1,0,0,0}; vGyroDriftEstimate = {{0,0,0}};
    std::memset(rotationFilters_, 0, sizeof(rotationFilters_));
    lastAccWeight = 0; recoveries = 0; computeRotationMatrix();
}
void InavCore::computeRotationMatrix()
{
    float q1q1 = orientation.q1 * orientation.q1;
    float q2q2 = orientation.q2 * orientation.q2;
    float q3q3 = orientation.q3 * orientation.q3;

    float q0q1 = orientation.q0 * orientation.q1;
    float q0q2 = orientation.q0 * orientation.q2;
    float q0q3 = orientation.q0 * orientation.q3;
    float q1q2 = orientation.q1 * orientation.q2;
    float q1q3 = orientation.q1 * orientation.q3;
    float q2q3 = orientation.q2 * orientation.q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}
void InavCore::integrate(float dt, const fpVector3_t& gyroRad,
                         const fpVector3_t* accBF, float accWScaler) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(accWScaler) ||
        !std::isfinite(gyroRad.x) || !std::isfinite(gyroRad.y) || !std::isfinite(gyroRad.z) ||
        (accBF && (!std::isfinite(accBF->x) || !std::isfinite(accBF->y) || !std::isfinite(accBF->z)))) {
        ++recoveries; return;
    }
    const fpQuaternion_t previous = orientation;
    const fpVector3_t previousDrift = vGyroDriftEstimate;
    fpVector3_t vRotation = gyroRad;
    const float spin_rate_sq = vectorNormSquared(&vRotation);
    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (accBF) {
        static const fpVector3_t vGravity = { .v = { 0.0f, 0.0f, 1.0f } };
        fpVector3_t vEstGravity, vAcc, vErr;

        // Calculate estimated gravity vector in body frame
        quaternionRotateVector(&vEstGravity, &vGravity, &orientation);    // EF -> BF

        // Error is sum of cross product between estimated direction and measured direction of gravity
        vectorNormalize(&vAcc, accBF);
        vectorCrossProduct(&vErr, &vAcc, &vEstGravity);

        // Compute and apply integral feedback if enabled
        if (imuRuntimeConfig.dcm_ki_acc > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate_sq < inavSquare(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT))) {
                fpVector3_t vTmp;

                // integral error scaled by Ki
                vectorScale(&vTmp, &vErr, imuRuntimeConfig.dcm_ki_acc * accWScaler * dt);
                vectorAdd(&vGyroDriftEstimate, &vGyroDriftEstimate, &vTmp);
            }
        }

        // Calculate kP gain and apply proportional feedback
        vectorScale(&vErr, &vErr, imuRuntimeConfig.dcm_kp_acc * accWScaler);
        vectorAdd(&vRotation, &vRotation, &vErr);
    }
    // Anti wind-up
    float i_limit = DEGREES_TO_RADIANS(2.0f) * (imuRuntimeConfig.dcm_kp_acc + imuRuntimeConfig.dcm_kp_mag) / 2.0f;
    vGyroDriftEstimate.x = constrainf(vGyroDriftEstimate.x, -i_limit, i_limit);
    vGyroDriftEstimate.y = constrainf(vGyroDriftEstimate.y, -i_limit, i_limit);
    vGyroDriftEstimate.z = constrainf(vGyroDriftEstimate.z, -i_limit, i_limit);

    // Apply gyro drift correction
    vectorAdd(&vRotation, &vRotation, &vGyroDriftEstimate);

    // Integrate rate of change of quaternion
    fpVector3_t vTheta;
    fpQuaternion_t deltaQ;

    vectorScale(&vTheta, &vRotation, 0.5f * dt);
    quaternionInitFromVector(&deltaQ, &vTheta);
    const float thetaMagnitudeSq = vectorNormSquared(&vTheta);

    // If calculated rotation is zero - don't update quaternion
    if (thetaMagnitudeSq >= 1e-20f) {
        // Calculate quaternion delta:
        // Theta is a axis/angle rotation. Direction of a vector is axis, magnitude is angle/2.
        // Proper quaternion from axis/angle involves computing sin/cos, but the formula becomes numerically unstable as Theta approaches zero.
        // For near-zero cases we use the first 3 terms of the Taylor series expansion for sin/cos. We check if fourth term is less than machine precision -
        // then we can safely use the "low angle" approximated version without loss of accuracy.
        if (thetaMagnitudeSq < fast_fsqrtf(24.0f * 1e-6f)) {
            quaternionScale(&deltaQ, &deltaQ, 1.0f - thetaMagnitudeSq / 6.0f);
            deltaQ.q0 = 1.0f - thetaMagnitudeSq / 2.0f;
        }
        else {
            const float thetaMagnitude = fast_fsqrtf(thetaMagnitudeSq);
            quaternionScale(&deltaQ, &deltaQ, sin_approx(thetaMagnitude) / thetaMagnitude);
            deltaQ.q0 = cos_approx(thetaMagnitude);
        }

        // Calculate final orientation and renormalize
        quaternionMultiply(&orientation, &orientation, &deltaQ);
        quaternionNormalize(&orientation, &orientation);
    }


    // Port safety guard: restore the previous state on invalid arithmetic.
    const float norm = quaternionNormSqared(&orientation);
    if (!std::isfinite(norm) || norm < 1e-12f) {
        orientation = previous; vGyroDriftEstimate = previousDrift; ++recoveries;
    }
    computeRotationMatrix();
}
void InavCore::step(const fpVector3_t& gyroRad, const fpVector3_t& accCmss,
                    float dt, uint32_t uptimeMs, bool armed) {
    float rotationSq = 0;
    for (unsigned axis=0; axis<3; ++axis) {
        float r = pt1FilterApply4(&rotationFilters_[axis], gyroRad.v[axis], 3, dt);
        rotationSq += r*r;
    }
    fpVector3_t normalized;
    vectorScale(&normalized, &accCmss, 1.0f / Gravity);
    float weight = bellCurve(fast_fsqrtf(vectorNormSquared(&normalized))-1.0f, 0.2f);
    weight *= (!armed && uptimeMs < 20000) ? 10.0f : 1.0f;
    if (armed) {
        const float rate = fast_fsqrtf(rotationSq) / 1.001f;
        weight *= scaleRangef(constrainf(rate,inavRadians(10),inavRadians(20)),inavRadians(10),inavRadians(20),1,0);
    }
    lastAccWeight = weight;
    integrate(dt,gyroRad,weight > 0.001f ? &accCmss : nullptr,weight);
}
void InavCore::zeroYaw() {
    const float angle = -atan2_approx(rMat[1][0],rMat[0][0]);
    fpQuaternion_t offset = {cos_approx(angle*0.5f),0,0,sin_approx(angle*0.5f)};
    quaternionMultiply(&orientation,&offset,&orientation);
    quaternionNormalize(&orientation,&orientation);
    computeRotationMatrix();
}
}}
