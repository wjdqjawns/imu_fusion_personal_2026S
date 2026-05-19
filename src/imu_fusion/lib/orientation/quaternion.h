#pragma once
#include "types.h"

Quat quatIdentity();
float quatNorm(const Quat& q);
Quat quatNormalize(const Quat& q);
Quat quatConj(const Quat& q);
Quat quatMul(const Quat& p, const Quat& q);

// Integrate quaternion with body angular rates [rad/s] over dt [s]
// Uses: q_dot = 0.5 * q * [0, gx, gy, gz]
Quat quatIntegrate(const Quat& q, float gx, float gy, float gz, float dt);
