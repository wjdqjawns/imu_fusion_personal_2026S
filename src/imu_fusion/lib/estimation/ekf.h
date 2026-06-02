#pragma once
#include "types.h"
#include "global_var.h"

// 6-state EKF: x = [phi, theta, psi, b_gx, b_gy, b_gz]^T
//   phi,theta,psi : roll/pitch/yaw [rad], world frame, ZYX aerospace
//   b_g{xyz}      : gyro bias [rad/s], random-walk model
typedef struct {
    float x[6];      // state vector
    float P[6][6];   // error covariance
    float mx_ref;    // world-frame mag reference [uT]
    float my_ref;
    float mz_ref;
} EkfState;

// phi0,theta0,psi0 [rad]; mx_ref,my_ref,mz_ref world-frame [uT]
void ekfInit(EkfState& s, float phi0, float theta0, float psi0,
             float mx_ref, float my_ref, float mz_ref);

// gx_rad,gy_rad,gz_rad [rad/s]; dt [s]
void ekfPredict(EkfState& s, float gx_rad, float gy_rad, float gz_rad, float dt);

// ax,ay,az [g]
void ekfUpdateAccel(EkfState& s, float ax, float ay, float az);

// mx,my,mz [uT]
void ekfUpdateMag(EkfState& s, float mx, float my, float mz);

EulerAngle ekfGetEuler(const EkfState& s);
