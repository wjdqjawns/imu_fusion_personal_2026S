#include "imu_sensor.h"

void cImuSensor::begin()
{
    Wire.begin();
    _imu.setWire(&Wire);
    _imu.beginAccel();
    _imu.beginGyro();
    _imu.beginMag();
}

uint8_t cImuSensor::readId()
{
    uint8_t id = 0;
    _imu.readId(&id);
    return id;
}

bool cImuSensor::update()
{
    uint8_t r = 0;
    r |= _imu.accelUpdate();
    r |= _imu.gyroUpdate();
    r |= _imu.magUpdate();
    return (r == 0);
}

sImuData cImuSensor::read()
{
    sImuData d;
    d.ax = _imu.accelX(); d.ay = _imu.accelY(); d.az = _imu.accelZ();
    d.gx = _imu.gyroX();  d.gy = _imu.gyroY();  d.gz = _imu.gyroZ();
    d.mx = _imu.magX();   d.my = _imu.magY();   d.mz = _imu.magZ();
    return d;
}
