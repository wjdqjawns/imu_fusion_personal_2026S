#include "imu_sensor.h"

void cImuSensor::begin()
{
    Wire.begin();
    _imu.setWire(&Wire);
    _imu.beginAccel();
    _imu.beginGyro();
    _imu.beginMag(MAG_MODE_CONTINUOUS_100HZ);  // default is 8Hz — must override for 100Hz output
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
    // Remap to NED body frame (X-forward, Y-right, Z-down).
    //
    // The library negates raw ADC: accelGet/gyroGet use ((float)-v).
    // Assuming chip is mounted with X-forward, Y-left, Z-up:
    //   negate all three axes to get X-fwd(+), Y-right(+), Z-down(+).
    //   At rest: az = -(-1g) = +1g, matches NED convention in euler.h.
    //
    // AK8963 is rotated inside the MPU9250 package (per datasheet):
    //   AK8963 X → MPU9250 Y (body left)  → my_ned = -magX()
    //   AK8963 Y → MPU9250 X (body fwd)   → mx_ned =  magY()
    //   AK8963 Z → MPU9250 -Z (body down) → mz_ned =  magZ()
    //
    // VERIFY against your actual physical mounting before using filters.
    d.ax = -_imu.accelX();
    d.ay = -_imu.accelY();
    d.az = -_imu.accelZ();
    d.gx = -_imu.gyroX();
    d.gy = -_imu.gyroY();
    d.gz = -_imu.gyroZ();
    d.mx =  _imu.magY();
    d.my = -_imu.magX();
    d.mz =  _imu.magZ();
    return d;
}
