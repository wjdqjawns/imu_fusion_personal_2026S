#pragma once
#include "types.h"

//------------------------------------------------------------
// Mode selection — define exactly ONE
//------------------------------------------------------------
#define MODE_RAW
// #define MODE_CALIBRATION
// #define MODE_STATE_ESTIMATION

//------------------------------------------------------------
// Filter selection (MODE_STATE_ESTIMATION)
//
// Define any combination. All active filters run on the SAME
// sensor data each loop, enabling fair single-session comparison.
//
//   0: FILTER_GYRO_ONLY    pure gyro integration — observe drift
//   1: FILTER_EKF          2-state EKF with gyro-bias estimation
//   2: FILTER_COMPLEMENTARY complementary filter
//   3: FILTER_MAHONY        Mahony nonlinear complementary
//   4: FILTER_MADGWICK      Madgwick gradient descent
//------------------------------------------------------------
#define FILTER_GYRO_ONLY
#define FILTER_EKF
// #define FILTER_COMPLEMENTARY
// #define FILTER_MAHONY
// #define FILTER_MADGWICK

//------------------------------------------------------------
// Timing
//------------------------------------------------------------
constexpr float Ts        = 0.01f;    // sampling period [s] = 100 Hz
constexpr int   LOOP_MS   = 10;       // loop delay [ms]
constexpr int   CALIB_N   = 500;      // calibration sample count
constexpr int   CALIB_DLY = 5;        // delay between cal samples [ms]
constexpr long  SERIAL_BAUD = 115200;

//------------------------------------------------------------
// Math constants
//------------------------------------------------------------
constexpr float RAD2DEG = 57.29577951f;
constexpr float DEG2RAD = 0.01745329f;

//------------------------------------------------------------
// EKF tuning
// Q_ANGLE: process noise — angle state  [rad^2/step]
// Q_BIAS:  process noise — gyro bias    [rad^2/s^2/step]
// R_ANGLE: measurement noise from accel [rad^2]
//------------------------------------------------------------
constexpr float EKF_Q_ANGLE = 0.001f;
constexpr float EKF_Q_BIAS  = 0.003f;
constexpr float EKF_R_ANGLE = 0.03f;

//------------------------------------------------------------
// Complementary filter
//------------------------------------------------------------
constexpr float CF_ALPHA = 0.98f;     // gyro trust weight (0=accel, 1=gyro)

//------------------------------------------------------------
// Mahony filter
//------------------------------------------------------------
constexpr float MAHONY_KP = 2.0f;
constexpr float MAHONY_KI = 0.005f;

//------------------------------------------------------------
// Madgwick filter
//------------------------------------------------------------
constexpr float MADGWICK_BETA = 0.1f;

//------------------------------------------------------------
// Global state (definitions in src/global_var.cpp)
//------------------------------------------------------------
extern ImuData    g_imuRaw;   // latest raw sensor reading
extern ImuData    g_imuCal;   // bias-corrected reading
extern ImuBias    g_bias;     // calibration bias