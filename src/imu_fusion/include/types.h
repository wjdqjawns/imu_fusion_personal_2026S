#pragma once
#include <Arduino.h>

// IMU sensor reading (raw or bias-corrected, same layout)
typedef struct sImuMeasurement
{
    float ax, ay, az;   // accelerometer [g]
    float gx, gy, gz;   // gyroscope     [rad/s]
    float mx, my, mz;   // magnetometer  [uT]
} sImuMeasurement;

// Per-axis bias from calibration
typedef struct sImuNoise
{
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
typedef struct sEulerAngle
{
    float phi;
    float theta;
    float psi;
} sEulerAngle;

// Unit quaternion: q = w + xi + yj + zk
typedef struct sQuat
{
    float w, x, y, z;
} sQuat;

// Direction Cosine Matrix: row-major 3x3
// Access element at row r, col c: R.m[r*3 + c]
typedef struct sDcm
{
    float m[9];
} sDcm;

static constexpr float PI_F = PI;