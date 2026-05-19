#pragma once
#include "types.h"
#include "global_var.h"

// 2-state scalar Kalman filter: x = [angle, gyro_bias]
// System model:
//   angle_{k+1} = angle_k + (rate - bias) * dt
//   bias_{k+1}  = bias_k
// Measurement: z = accel-derived angle
// State transition: A = [[1, -dt], [0, 1]]
typedef struct {
    float angle;      // estimated angle [rad]
    float bias;       // estimated gyro bias [rad/s]
    float P[2][2];    // 2x2 error covariance matrix
    float q_angle;    // process noise: angle variance [rad^2/step]
    float q_bias;     // process noise: bias variance  [rad^2/s^2/step]
    float r_angle;    // measurement noise [rad^2]
} EkfAxis;

typedef struct {
    EkfAxis phi;    // roll  axis driven by gx
    EkfAxis theta;  // pitch axis driven by gy
} EkfState;

void ekfInit(EkfState& s, float phi0, float theta0);

// gx,gy,gz [rad/s], ax,ay,az [g], dt [s]
void ekfUpdate(EkfState& s, float gx, float gy, float gz,
               float ax, float ay, float az, float dt);

EulerAngle ekfGetEuler(const EkfState& s);
