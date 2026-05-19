#pragma once
#include <Arduino.h>

// IMU sensor reading (raw or bias-corrected, same layout)
typedef struct sImuMeasurement{
    float ax, ay, az;   // accelerometer [g]
    float gx, gy, gz;   // gyroscope     [rad/s]
    float mx, my, mz;   // magnetometer  [uT]
} sImuMeasurement;

// Per-axis bias from calibration
typedef struct sImuNoise{
    float ax, ay, az;   // accel bias [g]
    float gx, gy, gz;   // gyro bias  [rad/s]
    float mx, my, mz;   // mag bias   [uT]

    // stddev of measurement noise, for EKF process noise covariance
    float ax_noise, ay_noise, az_noise;   // accel noise stddev [g]
    float gx_noise, gy_noise, gz_noise;   // gyro noise stddev  [rad/s]
    float mx_noise, my_noise, mz_noise;   // mag noise stddev
} sImuNoise;

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