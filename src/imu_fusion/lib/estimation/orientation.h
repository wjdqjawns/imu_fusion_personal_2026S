#pragma once

#include <math.h>
#include "types.h"

#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0f)
#endif
#ifndef RAD2DEG
#define RAD2DEG (180.0f / M_PI)
#endif

void accelAngles(const ImuData& d, float& phi_deg, float& theta_deg);
void gyroRates(const ImuData& d, float phi_deg, float theta_deg, float& phi_dot, float& theta_dot);
void euler2rot(const EulerAngle& e, float R[3][3]);

Quaternion rot2quat(const float R[3][3]);
Quaternion euler2quat(const EulerAngle& e);
EulerAngle quat2euler(const Quaternion& q);
Quaternion quatMul(const Quaternion& p, const Quaternion& q);
Quaternion quatNormalize(const Quaternion& q);