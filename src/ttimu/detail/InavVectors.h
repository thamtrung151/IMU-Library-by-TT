// SPDX-License-Identifier: GPL-3.0-or-later
// Adapted from INAV 9.0.1, d44f2cf6e60cc9318b10551291b596370b5c957b; see THIRD_PARTY_NOTICES.md.
#pragma once
#include "InavMath.h"
namespace ttimu { namespace detail {
static inline void vectorZero(fpVector3_t * v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

static inline float vectorNormSquared(const fpVector3_t * v)
{
    return inavSquare(v->x) + inavSquare(v->y) + inavSquare(v->z);
}

static inline fpVector3_t * vectorNormalize(fpVector3_t * result, const fpVector3_t * v)
{
    float length = fast_fsqrtf(vectorNormSquared(v));
    if (length != 0) {
        result->x = v->x / length;
        result->y = v->y / length;
        result->z = v->z / length;
    }
    else {
        result->x = 0;
        result->y = 0;
        result->z = 0;
    }
    return result;
}

static inline fpVector3_t * vectorCrossProduct(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->y * b->z - a->z * b->y;
    ab.y = a->z * b->x - a->x * b->z;
    ab.z = a->x * b->y - a->y * b->x;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorAdd(fpVector3_t * result, const fpVector3_t * a, const fpVector3_t * b)
{
    fpVector3_t ab;

    ab.x = a->x + b->x;
    ab.y = a->y + b->y;
    ab.z = a->z + b->z;

    *result = ab;
    return result;
}

static inline fpVector3_t * vectorScale(fpVector3_t * result, const fpVector3_t * a, const float b)
{
    fpVector3_t ab;

    ab.x = a->x * b;
    ab.y = a->y * b;
    ab.z = a->z * b;

    *result = ab;
    return result;
}

static inline fpQuaternion_t * quaternionInitUnit(fpQuaternion_t * result)
{
    result->q0 = 1.0f;
    result->q1 = 0.0f;
    result->q2 = 0.0f;
    result->q3 = 0.0f;
    return result;
}

static inline fpQuaternion_t * quaternionInitFromVector(fpQuaternion_t * result, const fpVector3_t * v)
{
    result->q0 = 0.0f;
    result->q1 = v->x;
    result->q2 = v->y;
    result->q3 = v->z;
    return result;
}

static inline float quaternionNormSqared(const fpQuaternion_t * q)
{
    return inavSquare(q->q0) + inavSquare(q->q1) + inavSquare(q->q2) + inavSquare(q->q3);
}

static inline fpQuaternion_t * quaternionMultiply(fpQuaternion_t * result, const fpQuaternion_t * a, const fpQuaternion_t * b)
{
  fpQuaternion_t p;

  p.q0 = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
  p.q1 = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
  p.q2 = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
  p.q3 = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;

  *result = p;
  return result;
}

static inline fpQuaternion_t * quaternionScale(fpQuaternion_t * result, const fpQuaternion_t * a, const float b)
{
    fpQuaternion_t p;

    p.q0 = a->q0 * b;
    p.q1 = a->q1 * b;
    p.q2 = a->q2 * b;
    p.q3 = a->q3 * b;

    *result = p;
    return result;
}

static inline fpQuaternion_t * quaternionConjugate(fpQuaternion_t * result, const fpQuaternion_t * q)
{
    result->q0 =  q->q0;
    result->q1 = -q->q1;
    result->q2 = -q->q2;
    result->q3 = -q->q3;

    return result;
}

static inline fpQuaternion_t * quaternionNormalize(fpQuaternion_t * result, const fpQuaternion_t * q)
{
    float mod = fast_fsqrtf(quaternionNormSqared(q));
    if (mod < 1e-6f) {
        // Length is too small - re-initialize to zero rotation
        result->q0 = 1;
        result->q1 = 0;
        result->q2 = 0;
        result->q3 = 0;
    }
    else {
        result->q0 = q->q0 / mod;
        result->q1 = q->q1 / mod;
        result->q2 = q->q2 / mod;
        result->q3 = q->q3 / mod;
    }

    return result;
}

static inline fpVector3_t * quaternionRotateVector(fpVector3_t * result, const fpVector3_t * vect, const fpQuaternion_t * ref)
{
    fpQuaternion_t vectQuat, refConj;

    vectQuat.q0 = 0;
    vectQuat.q1 = vect->x;
    vectQuat.q2 = vect->y;
    vectQuat.q3 = vect->z;

    quaternionConjugate(&refConj, ref);
    quaternionMultiply(&vectQuat, &refConj, &vectQuat);
    quaternionMultiply(&vectQuat, &vectQuat, ref);

    result->x = vectQuat.q1;
    result->y = vectQuat.q2;
    result->z = vectQuat.q3;
    return result;
}

static inline fpVector3_t * quaternionRotateVectorInv(fpVector3_t * result, const fpVector3_t * vect, const fpQuaternion_t * ref)
{
    fpQuaternion_t vectQuat, refConj;

    vectQuat.q0 = 0;
    vectQuat.q1 = vect->x;
    vectQuat.q2 = vect->y;
    vectQuat.q3 = vect->z;

    quaternionConjugate(&refConj, ref);
    quaternionMultiply(&vectQuat, ref, &vectQuat);
    quaternionMultiply(&vectQuat, &vectQuat, &refConj);

    result->x = vectQuat.q1;
    result->y = vectQuat.q2;
    result->z = vectQuat.q3;
    return result;
}
}}
