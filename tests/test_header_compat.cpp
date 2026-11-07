// SPDX-License-Identifier: GPL-3.0-or-later
// Regression: a normal consumer includes Arduino.h before IMULibrary.h.
#include <cmath>
#define sq(x) ((x)*(x))
#define radians(deg) ((deg)*0.017453292519943295769236907684886)
#include "IMULibrary.h"

#ifndef sq
#error "The library must preserve Arduino's sq macro"
#endif
#ifndef radians
#error "The library must preserve Arduino's radians macro"
#endif
static_assert(sq(3) == 9, "Arduino macro still usable after the public header");
static_assert(radians(0) == 0, "Arduino macro still usable after the public header");
int main() { ttimu::IMU imu; return imu.ready() ? 1 : 0; }
