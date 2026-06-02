#pragma once

#include "types.h"
#include "imu_sensor.h"

sImuBias calibrate(cImuSensor& imu, int N = 1000, int delayMs = 3);
sImuData applyBias(const sImuData& raw, const sImuBias& bias);
void     printBias(const sImuBias& b);
