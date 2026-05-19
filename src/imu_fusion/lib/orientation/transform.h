#pragma once
#include "types.h"

// All angles in [rad], ZYX aerospace convention (yaw→pitch→roll)

// Euler ↔ Quaternion
Quat       euler2quat(const EulerAngle& e);
EulerAngle quat2euler(const Quat& q);

// Euler ↔ DCM
Dcm        euler2dcm(const EulerAngle& e);
EulerAngle dcm2euler(const Dcm& R);

// DCM ↔ Quaternion (Shepperd's method — numerically stable)
Quat dcm2quat(const Dcm& R);
Dcm  quat2dcm(const Quat& q);
