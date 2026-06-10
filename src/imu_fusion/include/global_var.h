#pragma once
#include "types.h"

//------------------------------------------------------------
// Mode selection — define exactly ONE
//------------------------------------------------------------
#define MODE_RAW
// #define MODE_CALIBRATION
// #define MODE_STATE_ESTIMATION         // CSV text, all 5 filters
// #define MODE_STATE_ESTIMATION_BINARY  // binary framed, all 5 filters

//------------------------------------------------------------
// Data Framing
//------------------------------------------------------------
constexpr uint16_t FRAME_HEADER = 0xAA55;

//------------------------------------------------------------
// Timing
//------------------------------------------------------------
constexpr float    Ts         = 0.01f;
constexpr int      LOOP_MS    = 10;
constexpr uint32_t LOOP_US    = 10000UL;   // 10ms in microseconds
constexpr int      CALIB_N    = 500;
constexpr int      CALIB_DLY  = 5;
constexpr long     SERIAL_BAUD = 230400;  // CSV@100Hz ≈18KB/s; needs ≥230400

//------------------------------------------------------------
// Math constants
//------------------------------------------------------------
constexpr float RAD2DEG = 57.29577951f;
constexpr float DEG2RAD = 0.01745329f;

//------------------------------------------------------------
// EKF tuning (6-state: x = [phi,theta,psi, b_gx,b_gy,b_gz])
//------------------------------------------------------------
constexpr float EKF_Q_ANGLE  = 0.001f;   // process noise: angle   [rad^2/step]
constexpr float EKF_Q_BIAS   = 0.003f;   // process noise: gyro bias[rad^2/s^2/step]
constexpr float EKF_R_ACCEL  = 0.03f;    // accel measurement noise [m^2/s^4]
constexpr float EKF_R_MAG    = 0.1f;     // mag measurement noise   [uT^2]
constexpr float ACCEL_THRESH = 1.5f;     // dynamic-motion guard    [m/s^2]
constexpr float MAG_THRESH   = 5.0f;     // mag-disturbance guard   [uT]

//------------------------------------------------------------
// Complementary filter
//------------------------------------------------------------
constexpr float CF_ALPHA = 0.98f;

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
extern sMsgPacket g_msg_packet;
extern sImuData   g_imuRaw;
extern sImuBias   g_bias;