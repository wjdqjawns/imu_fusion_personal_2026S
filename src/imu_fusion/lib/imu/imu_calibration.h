#pragma once

#include "types.h"
#include "imu_sensor.h"

// Gyro bias + accel flat-rest offset.
// Sensor must be flat and still. N samples at delayMs intervals.
// After applyBias: gx/gy/gz ≈ 0, ax/ay ≈ 0, az ≈ +1g (NED gravity preserved).
sImuBias calibrateStaticBias(cImuSensor& imu, int N = 500, int delayMs = 3);

// Magnetometer hard-iron offset via min-max sweep.
// Rotate sensor through all orientations during durationMs.
// After applyBias: mag sphere centered at origin.
sImuBias calibrateMagBias(cImuSensor& imu, uint32_t durationMs = 30000);

// Apply bias to a raw reading.
sImuData applyBias(const sImuData& raw, const sImuBias& bias);

// Print full bias as a C initializer to paste into global_var.cpp.
void printBias(const sImuBias& b);
