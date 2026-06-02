#pragma once
#include <Arduino.h>

#pragma pack(push, 1)
typedef struct sMsgPacket
{
    uint16_t header;   // 0xAA55
    uint16_t length;   // bytes from seq to just before crc (42 fixed)
    uint16_t seq;
    uint32_t time_us;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
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

constexpr uint16_t MSG_PAYLOAD_LEN = sizeof(sMsgPacket)
    - sizeof(sMsgPacket::header)
    - sizeof(sMsgPacket::length)
    - sizeof(sMsgPacket::seq)
    - sizeof(sMsgPacket::crc);