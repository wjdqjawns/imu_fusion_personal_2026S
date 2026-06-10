#pragma once
#include "types.h"

// Pure gyroscope integration — no measurement correction.
// Purpose: observe raw drift and compare kinematic representations.
//
// Three representations updated in parallel each step:
//   euler : ZYX Euler kinematics  (suffers gimbal lock near pitch ±90°)
//   quat  : Quaternion kinematics (no gimbal lock, requires normalization)
//   dcm   : DCM kinematics        (no gimbal lock, requires orthonormalization)
typedef struct sGyroPropState
{
    EulerAngle euler;
    Quat       quat;
    Dcm        dcm;
} sGyroPropState;
typedef sGyroPropState GyroPropState;

void gyroPropInit(sGyroPropState& s);

// Update all three representations with body rates [rad/s] and dt [s]
void gyroPropUpdate(sGyroPropState& s, float gx, float gy, float gz, float dt);

// Retrieve Euler angles from each representation for comparison
EulerAngle gyroPropGetEulerFromEuler(const sGyroPropState& s);
EulerAngle gyroPropGetEulerFromQuat(const sGyroPropState& s);
EulerAngle gyroPropGetEulerFromDcm(const sGyroPropState& s);