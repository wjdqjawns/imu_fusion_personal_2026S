#include "imu_calibration.h"
#include <Arduino.h>

sImuBias calibrate(cImuSensor& imu, int N, int delayMs)
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
    b.az = (float)(az_s / N);  // az expected ≈ -1 g when flat
    b.gx = (float)(gx_s / N);
    b.gy = (float)(gy_s / N);
    b.gz = (float)(gz_s / N);
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
    Serial.println(F("===== Calibration Bias ====="));
    Serial.print(F("ax=")); Serial.print(b.ax, 6);
    Serial.print(F(" ay=")); Serial.print(b.ay, 6);
    Serial.print(F(" az=")); Serial.println(b.az, 6);
    Serial.print(F("gx=")); Serial.print(b.gx, 6);
    Serial.print(F(" gy=")); Serial.print(b.gy, 6);
    Serial.print(F(" gz=")); Serial.println(b.gz, 6);
    Serial.println(F("===== Paste into global_var.cpp ====="));
    Serial.print(F(".ax=")); Serial.print(b.ax, 6); Serial.print(F(", "));
    Serial.print(F(".ay=")); Serial.print(b.ay, 6); Serial.print(F(", "));
    Serial.print(F(".az=")); Serial.println(b.az, 6);
    Serial.print(F(".gx=")); Serial.print(b.gx, 6); Serial.print(F(", "));
    Serial.print(F(".gy=")); Serial.print(b.gy, 6); Serial.print(F(", "));
    Serial.print(F(".gz=")); Serial.println(b.gz, 6);
}
