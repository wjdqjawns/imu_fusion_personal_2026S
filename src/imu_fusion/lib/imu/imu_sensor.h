#pragma once

#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include "types.h"

class cImuSensor
{
public:
    void begin();
    uint8_t readId(); // sensor id read
    bool update();    // sensor update, return true if success
    sImuData read();   // return latest sensor data (after update)

private:
    MPU9250_asukiaaa _imu;
};