#pragma once

#include "types.h"
#include "imu_sensor.h"

ImuBias calibrate(ImuSensor& imu, int N = 1000, int delayMs = 3);
ImuData applyBias(const ImuData& raw, const ImuBias& bias);

void printBias(const ImuBias& b);