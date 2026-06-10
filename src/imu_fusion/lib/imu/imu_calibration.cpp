#include "imu_calibration.h"
#include <Arduino.h>

sImuBias calibrateStaticBias(cImuSensor& imu, int N, int delayMs)
{
    double ax_s = 0, ay_s = 0, az_s = 0;
    double gx_s = 0, gy_s = 0, gz_s = 0;

    for (int i = 0; i < N; i++)
    {
        imu.update();
        sImuData d = imu.read();
        ax_s += d.ax; ay_s += d.ay; az_s += d.az;
        gx_s += d.gx; gy_s += d.gy; gz_s += d.gz;
        delay(delayMs);
    }

    sImuBias b = {};
    b.ax = (float)(ax_s / N);
    b.ay = (float)(ay_s / N);
    b.az = (float)(az_s / N) - 1.0f;  // preserve +1g NED gravity: bias = mean - 1g
    b.gx = (float)(gx_s / N);
    b.gy = (float)(gy_s / N);
    b.gz = (float)(gz_s / N);
    // mx/my/mz left zero — filled separately by calibrateMagBias
    return b;
}

sImuBias calibrateMagBias(cImuSensor& imu, uint32_t durationMs)
{
    float mx_min =  1e6f, my_min =  1e6f, mz_min =  1e6f;
    float mx_max = -1e6f, my_max = -1e6f, mz_max = -1e6f;

    uint32_t t_start = millis();
    while ((millis() - t_start) < durationMs)
    {
        imu.update();
        sImuData d = imu.read();
        if (d.mx < mx_min) mx_min = d.mx;
        if (d.mx > mx_max) mx_max = d.mx;
        if (d.my < my_min) my_min = d.my;
        if (d.my > my_max) my_max = d.my;
        if (d.mz < mz_min) mz_min = d.mz;
        if (d.mz > mz_max) mz_max = d.mz;
        delay(10);
    }

    sImuBias b = {};
    b.mx = (mx_max + mx_min) * 0.5f;
    b.my = (my_max + my_min) * 0.5f;
    b.mz = (mz_max + mz_min) * 0.5f;
    return b;
}

sImuData applyBias(const sImuData& raw, const sImuBias& b)
{
    sImuData cal = raw;
    cal.ax -= b.ax; cal.ay -= b.ay; cal.az -= b.az;
    cal.gx -= b.gx; cal.gy -= b.gy; cal.gz -= b.gz;
    cal.mx -= b.mx; cal.my -= b.my; cal.mz -= b.mz;
    return cal;
}

void printBias(const sImuBias& b)
{
    Serial.println(F("===== PASTE INTO src/global_var.cpp ====="));
    Serial.println(F("sImuBias g_bias = {"));
    Serial.print(F("    .ax=")); Serial.print(b.ax,6); Serial.print(F("f, "));
    Serial.print(F(".ay=")); Serial.print(b.ay,6); Serial.print(F("f, "));
    Serial.print(F(".az=")); Serial.print(b.az,6); Serial.println(F("f,"));
    Serial.print(F("    .gx=")); Serial.print(b.gx,6); Serial.print(F("f, "));
    Serial.print(F(".gy=")); Serial.print(b.gy,6); Serial.print(F("f, "));
    Serial.print(F(".gz=")); Serial.print(b.gz,6); Serial.println(F("f,"));
    Serial.print(F("    .mx=")); Serial.print(b.mx,3); Serial.print(F("f, "));
    Serial.print(F(".my=")); Serial.print(b.my,3); Serial.print(F("f, "));
    Serial.print(F(".mz=")); Serial.print(b.mz,3); Serial.println(F("f"));
    Serial.println(F("};"));
    Serial.println(F("========================================="));
}