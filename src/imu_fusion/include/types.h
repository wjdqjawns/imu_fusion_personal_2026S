#pragma once
#include <Arduino.h>

// IMU sensor reading (raw or bias-corrected, same layout)
typedef struct {
    float ax, ay, az;   // accelerometer [g]
    float gx, gy, gz;   // gyroscope     [rad/s]
    float mx, my, mz;   // magnetometer  [uT]
} ImuData;

// Per-axis bias from calibration
typedef struct {
    float ax, ay, az;   // accel bias [g]
    float gx, gy, gz;   // gyro bias  [rad/s]
} ImuBias;

// Euler angles: ZYX aerospace convention, all in [rad]
// phi = roll, theta = pitch, psi = yaw
typedef struct {
    float phi;
    float theta;
    float psi;
} EulerAngle;

// Unit quaternion: q = w + xi + yj + zk
typedef struct {
    float w, x, y, z;
} Quat;

// Direction Cosine Matrix: row-major 3x3
// Access element at row r, col c: R.m[r*3 + c]
typedef struct {
    float m[9];
} Dcm;

static constexpr float PI_F = PI;