// SPDX-License-Identifier: GPL-3.0-or-later
// Types and primitives adapted from INAV 9.0.1. See THIRD_PARTY_NOTICES.md.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ttimu { namespace detail {
#ifndef M_PIf
// GNU libc may already provide the same float constant as a macro.
constexpr float M_PIf = 3.14159265358979323846f;
#endif
constexpr float Gravity = 980.665f;
constexpr unsigned MAX_KALMAN_WINDOW_SIZE = 64;
constexpr float VARIANCE_SCALE = 0.67f;
// Prefix helpers so Arduino's sq()/radians() macros can stay untouched.
template<class T> inline T inavSquare(T x) { return x * x; }
inline float inavRadians(float x) { return x * (M_PIf / 180.0f); }
typedef union { float v[3]; struct { float x, y, z; }; } fpVector3_t;
struct fpQuaternion_t { float q0, q1, q2, q3; };
struct stdev_t { float m_oldM, m_newM, m_oldS, m_newS; unsigned m_n; };
struct pt1Filter_t { float state, RC, dT, alpha; };
struct biquadFilter_t { float b0, b1, b2, a1, a2, x1, x2, y1, y2; };
enum biquadFilterType_e { FILTER_LPF, FILTER_NOTCH };
struct kalman_t {
    float q, r, p, k, x, lastX, e, setpoint, axisVar;
    uint16_t windex;
    float axisWindow[MAX_KALMAN_WINDOW_SIZE + 1];
    float varianceWindow[MAX_KALMAN_WINDOW_SIZE + 1];
    float axisSumMean, axisMean, axisSumVar, inverseN;
    uint16_t w;
};
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
float fast_fsqrtf(float value);
float constrainf(float amt, float low, float high);
float scaleRangef(float x, float srcMin, float srcMax, float destMin, float destMax);
float gaussian(float x, float mu, float sigma);
float bellCurve(float x, float width);
void devClear(stdev_t* dev);
void devPush(stdev_t* dev, float x);
float devVariance(stdev_t* dev);
float devStandardDeviation(stdev_t* dev);
void pt1FilterInitRC(pt1Filter_t*, float, float);
void pt1FilterInit(pt1Filter_t*, float, float);
float pt1FilterApply(pt1Filter_t*, float);
float pt1FilterApply4(pt1Filter_t*, float, float, float);
void pt1FilterReset(pt1Filter_t*, float);
void biquadFilterInit(biquadFilter_t*, uint16_t, uint32_t, float, biquadFilterType_e);
float biquadFilterApply(biquadFilter_t*, float);
float biquadFilterReset(biquadFilter_t*, float);
void gyroKalmanInitAxis(kalman_t*, uint16_t);
float kalman_process(kalman_t*, float);
void updateAxisVariance(kalman_t*, float);
}}
