#pragma once
#include "ttimu/detail/InavMath.h"
namespace reference {
void reset();
void step(const ttimu::detail::fpVector3_t& gyro, const ttimu::detail::fpVector3_t& acc,
          float dt,uint32_t uptimeMs,bool armed);
ttimu::detail::fpQuaternion_t quaternion();
ttimu::detail::fpVector3_t integral();
float weight();
}
