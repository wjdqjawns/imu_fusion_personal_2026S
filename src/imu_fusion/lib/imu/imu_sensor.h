#pragma once

#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include "types.h"

class ImuSensor
{
public:
    void begin(bool useMag = false);
    bool update();           // 센서 값 갱신, 실패 시 false
    ImuData read();    // 최신 값 반환

private:
    MPU9250_asukiaaa _imu;
    bool _useMag = false;
};