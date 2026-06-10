#pragma once
#include <Arduino.h>

// ─── Orientation math primitives ─────────────────────────────────────────────
typedef struct { float w, x, y, z; } Quat;
typedef struct { float phi, theta, psi; } EulerAngle;  // roll/pitch/yaw [rad]
typedef struct { float m[9]; } Dcm;                    // row-major 3×3

// Packet layout (108 bytes total):
//   header(2) | length(2) | payload(102) | crc(2)
//
// payload = seq(2) + time_us(4) + 9 raw floats(36) + 15 angle floats(60)
// crc covers header through end of payload (all bytes before crc field)
#pragma pack(push, 1)
typedef struct sMsgPacket
{
    uint16_t header;    // 0xAA55
    uint16_t length;    // payload bytes: from seq to last angle field (102)
    uint16_t seq;
    uint32_t time_us;
    // raw IMU
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    // gyro propagation
    float roll_gyro, pitch_gyro, yaw_gyro;
    // complementary filter
    float roll_cf, pitch_cf, yaw_cf;
    // EKF
    float roll_ekf, pitch_ekf, yaw_ekf;
    // Madgwick
    float roll_madgwick, pitch_madgwick, yaw_madgwick;
    // Mahony
    float roll_mahony, pitch_mahony, yaw_mahony;
    uint16_t crc;
} sMsgPacket;
#pragma pack(pop)

typedef struct sImuData
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
} sImuData;

typedef struct sImuBias
{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
} sImuBias;

// payload = everything between length field and crc field
constexpr uint16_t MSG_PAYLOAD_LEN = sizeof(sMsgPacket)
    - sizeof(sMsgPacket::header)
    - sizeof(sMsgPacket::length)
    - sizeof(sMsgPacket::crc);