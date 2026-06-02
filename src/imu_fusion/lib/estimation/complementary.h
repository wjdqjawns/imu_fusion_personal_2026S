#pragma once
#include "types.h"

// Complementary filter state (Euler angles [rad])
typedef struct
{
    float phi;    // roll  [rad]
    float theta;  // pitch [rad]
    float psi;    // yaw   [rad] — updated by mag if available
} CfState;

void cfInit(CfState& s, float phi0, float theta0, float psi0);

// Update with body rates [rad/s], accel [g], dt [s], alpha = gyro trust weight
// alpha = 0.98: 98% gyro, 2% accel correction per step
// No yaw correction without magnetometer; psi drifts freely.
void cfUpdate(CfState& s,
              float gx, float gy, float gz,
              float ax, float ay, float az,
              float dt, float alpha);

// Update with magnetometer [uT] for yaw correction (call after cfUpdate)
void cfUpdateYaw(CfState& s, float mx, float my, float mz, float alpha_yaw);

EulerAngle cfGetEuler(const CfState& s);