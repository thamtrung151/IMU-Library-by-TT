// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <cstdint>

namespace fc {
// Starting alignment from INAV 9 MATEKF411TE: IMU_ICM42605_ALIGN CW180_DEG_FLIP.
// This is NOT yet verified for your custom PCB, which needed manual INAV alignment.
// Check flat-board accel Z near +9.8 m/s^2 and roll/pitch directions before use.
// Preserve your INAV `diff all` before flashing; it may contain the needed rotation.
constexpr int8_t Axes[3] = {1, -2, -3};
}
