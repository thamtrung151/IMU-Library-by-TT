// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace fc {
// Custom PCB correction from Nov 7 logs: old Z was -g and roll/pitch were swapped.
// Relative to the old {1,-2,-3}: new body axes = {old Y, old X, -old Z}.
// This is a proper rotation (determinant +1), INAV CW270_DEG, not a left-handed swap.
// User motion test confirmed pitch/yaw signs; roll output convention is set below.
constexpr int8_t Axes[3] = {-2, 1, 3};
// User verified pitch/yaw signs; reverse only the reported roll convention.
constexpr int8_t EulerRollSign = -1;
}
