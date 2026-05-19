#pragma once
#include "types.h"

// Madgwick gradient descent filter (IMU mode, no magnetometer)
// Reference: Madgwick et al. 2010, "An efficient orientation filter for IMU and MARG"
//
// Minimizes the objective function:
//   f(q) = q* ⊗ g_earth ⊗ q - a_measured
// using gradient descent to correct the gyro-integrated quaternion.
// beta controls the gradient step size (larger = faster but noisier).
typedef struct {
    Quat q;   // current attitude quaternion
} MadgwickState;

void madgwickInit(MadgwickState& s);

// ax,ay,az [g], gx,gy,gz [rad/s], dt [s], beta from global_var.h
void madgwickUpdate(MadgwickState& s,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float dt, float beta);

EulerAngle madgwickGetEuler(const MadgwickState& s);
