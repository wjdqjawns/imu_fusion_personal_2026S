#include "imu_calibration.h"
#include <Arduino.h>

ImuBias calibrate(ImuSensor& imu, int N, int delayMs) {
    double ax_s = 0, ay_s = 0, az_s = 0;
    double gx_s = 0, gy_s = 0, gz_s = 0;

    for (int i = 0; i < N; i++) {
        imu.update();
        ImuData d = imu.read();
        ax_s += d.ax; ay_s += d.ay; az_s += d.az;
        gx_s += d.gx; gy_s += d.gy; gz_s += d.gz;
        delay(delayMs);
    }

    ImuBias b;
    b.ax = (float)(ax_s / N) - 0.0f;
    b.ay = (float)(ay_s / N) - 0.0f;
    b.az = (float)(az_s / N) - (-1.0f);  // az expected = -1g
    b.gx = (float)(gx_s / N);
    b.gy = (float)(gy_s / N);
    b.gz = (float)(gz_s / N);
    return b;
}

ImuData applyBias(const ImuData& raw, const ImuBias& b) {
    ImuData cal = raw;
    cal.ax -= b.ax; cal.ay -= b.ay; cal.az -= b.az;
    cal.gx -= b.gx; cal.gy -= b.gy; cal.gz -= b.gz;
    return cal;
}

void printBias(const ImuBias& b) {
    Serial.println(F("===== Calibration Bias ====="));
    Serial.print(F("ax_bias = ")); Serial.println(b.ax, 6);
    Serial.print(F("ay_bias = ")); Serial.println(b.ay, 6);
    Serial.print(F("az_bias = ")); Serial.println(b.az, 6);
    Serial.print(F("gx_bias = ")); Serial.println(b.gx, 6);
    Serial.print(F("gy_bias = ")); Serial.println(b.gy, 6);
    Serial.print(F("gz_bias = ")); Serial.println(b.gz, 6);
    Serial.println(F("===== Copy to globals.h ====="));
    Serial.print(F(".ax=")); Serial.print(b.ax,6); Serial.print(F(", "));
    Serial.print(F(".ay=")); Serial.print(b.ay,6); Serial.print(F(", "));
    Serial.print(F(".az=")); Serial.print(b.az,6); Serial.println();
    Serial.print(F(".gx=")); Serial.print(b.gx,6); Serial.print(F(", "));
    Serial.print(F(".gy=")); Serial.print(b.gy,6); Serial.print(F(", "));
    Serial.print(F(".gz=")); Serial.print(b.gz,6); Serial.println();
}
