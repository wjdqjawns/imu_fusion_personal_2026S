#include "imu_sensor.h"

void ImuSensor::begin(bool useMag)
{
    _useMag = useMag;
    Wire.begin();
    _imu.setWire(&Wire);
    _imu.beginAccel();
    _imu.beginGyro();
    if (_useMag) _imu.beginMag();
}

bool ImuSensor::update()
{
    uint8_t r = 0;
    r |= _imu.accelUpdate();
    r |= _imu.gyroUpdate();
    if (_useMag) r |= _imu.magUpdate();
    return (r == 0);
}

ImuData ImuSensor::read()
{
    ImuData d;
    d.ax = _imu.accelX(); d.ay = _imu.accelY(); d.az = _imu.accelZ();
    d.gx = _imu.gyroX();  d.gy = _imu.gyroY();  d.gz = _imu.gyroZ();
    d.mx = _imu.magX();   d.my = _imu.magY();   d.mz = _imu.magZ();
    return d;
}