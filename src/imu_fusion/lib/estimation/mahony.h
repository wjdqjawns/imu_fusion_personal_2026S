#pragma once
#include "types.h"

// Mahony nonlinear complementary filter (IMU mode, no magnetometer)
// Reference: Mahony et al. 2008, "Nonlinear Complementary Filters on SO(3)"
//
// Uses a PI controller in rotation space:
//   error  = estimated_gravity × measured_gravity  (cross product)
//   omega_corrected = omega + Kp*error + Ki*integral(error)
//   q updated by corrected omega
typedef struct {
    Quat  q;                    // current attitude quaternion
    float ex_int, ey_int, ez_int;  // integral error accumulator
} MahonyState;

void mahonyInit(MahonyState& s);

// ax,ay,az [g], gx,gy,gz [rad/s], dt [s], Kp/Ki from global_var.h
void mahonyUpdate(MahonyState& s,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dt, float Kp, float Ki);

EulerAngle mahonyGetEuler(const MahonyState& s);
