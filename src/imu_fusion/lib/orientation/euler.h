#pragma once
#include "types.h"

// ZYX Euler kinematic differential equations
// Maps body angular rates [gx,gy,gz] to Euler angle rates
// NOTE: psi_dot diverges near theta = ±90° (gimbal lock)
void eulerKinematics(float phi, float theta,
                     float gx, float gy, float gz,
                     float& phi_dot, float& theta_dot, float& psi_dot);

// Roll/pitch from static accelerometer reading [g] → [rad]
// Standard NED frame: az ≈ +1g at rest (gravity points into +Z body axis)
// Adjust signs in .cpp if your sensor mount differs
void accelRollPitch(float ax, float ay, float az, float& phi, float& theta);

// Yaw from magnetometer + known roll/pitch [rad] → [rad]
float magYaw(float mx, float my, float mz, float phi, float theta);
