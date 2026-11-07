// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace fc {
// Custom PCB correction from Nov 7 logs: old Z was -g and roll/pitch were swapped.
// Relative to the old {1,-2,-3}: new body axes = {old Y, old X, -old Z}.
// This is a proper rotation (determinant +1), INAV CW270_DEG, not a left-handed swap.
// Static logs cannot fix the remaining 180-degree heading ambiguity: verify signs
// with a nose-up and right-side-down test. CW90_DEG {2,-1,3} is the other sign choice.
constexpr int8_t Axes[3] = {-2, 1, 3};
}
