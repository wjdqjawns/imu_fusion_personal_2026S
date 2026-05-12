#pragma once

#include "types.h"

// mode selection
// #define MODE_RAW
// #define MODE_CALIBRATION
#define MODE_KALMAN

// sampling
constexpr float Ts        = 0.05f;    // sampling time [s]  → 20 Hz
constexpr int   LOOP_MS   = 50;       // delay per loop [ms]
constexpr int   CALIB_N   = 1000;     // calibration sample count
constexpr int   CALIB_DLY = 3;        // delay between calibration samples [ms]
constexpr long   SERIAL_BAUD = 115200;

// kalman filter noise parameters
constexpr float KF_Q_PHI   = 0.001f;  // process noise  – roll  [deg²]
constexpr float KF_Q_THETA = 0.001f;  // process noise  – pitch [deg²]
constexpr float KF_R_PHI   = 0.03f;   // measurement noise – roll  [deg²]
constexpr float KF_R_THETA = 0.03f;   // measurement noise – pitch [deg²]

// calibration bias
// MODE_CALIBRATION 실행 후 얻은 값을 여기에 붙여넣기
extern ImuBias g_bias;

// runtime state variables
extern ImuData    g_imuRaw;    // latest raw sensor reading
extern ImuData    g_imuCal;    // latest bias-compensated reading
extern EulerAngle g_euler;     // latest Kalman filter output