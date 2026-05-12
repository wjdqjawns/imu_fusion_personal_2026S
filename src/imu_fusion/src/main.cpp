#include <Arduino.h>
#include "global_var.h"
#include "imu_sensor.h"
#include "imu_calibration.h"
#include "kalman_filter.h"
#include "orientation.h"

ImuSensor imu;

#if defined(MODE_RAW)

void setup() {
    Serial.begin(SERIAL_BAUD);
    imu.begin(true);
    delay(1000);
    Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
    imu.update();
    g_imuRaw = imu.read();
    const ImuData& d = g_imuRaw;
    Serial.print(d.ax,4); Serial.print(",");
    Serial.print(d.ay,4); Serial.print(",");
    Serial.print(d.az,4); Serial.print(",");
    Serial.print(d.gx,4); Serial.print(",");
    Serial.print(d.gy,4); Serial.print(",");
    Serial.print(d.gz,4); Serial.print(",");
    Serial.print(d.mx,4); Serial.print(",");
    Serial.print(d.my,4); Serial.print(",");
    Serial.println(d.mz,4);
    delay(LOOP_MS);
}

#elif defined(MODE_CALIBRATION)

void setup() {
    Serial.begin(SERIAL_BAUD);
    imu.begin(false);
    delay(1000);
    Serial.println(F("=== IMU Calibration ==="));
    Serial.println(F("평평한 곳에 정지 상태로 놓으세요. 3초 후 시작..."));
    delay(3000);

    ImuBias b = calibrate(imu, CALIB_N, CALIB_DLY);
    printBias(b);
    Serial.println(F("→ 위 값을 src/globals.cpp 의 g_bias 에 복사하세요."));
}

void loop() {}

#elif defined(MODE_KALMAN)

KalmanFilter kf;

void setup() {
    Serial.begin(SERIAL_BAUD);
    imu.begin(false);
    delay(1000);

    // 첫 가속도계 값으로 KF 초기화
    imu.update();
    g_imuRaw = imu.read();
    g_imuCal = applyBias(g_imuRaw, g_bias);
    float phi0, theta0;
    accelAngles(g_imuCal, phi0, theta0);
    kf.init(phi0, theta0);

    Serial.println(F("time_ms,phi_kf,theta_kf,phi_acc,theta_acc"));
}

void loop() {
    imu.update();
    g_imuRaw = imu.read();
    g_imuCal = applyBias(g_imuRaw, g_bias);

    kf.update(g_imuCal, Ts);
    g_euler = kf.getEuler();

    float phi_acc, theta_acc;
    accelAngles(g_imuCal, phi_acc, theta_acc);

    Serial.print(millis());            Serial.print(F(","));
    Serial.print(g_euler.phi,   3);   Serial.print(F(","));
    Serial.print(g_euler.theta, 3);   Serial.print(F(","));
    Serial.print(phi_acc,       3);   Serial.print(F(","));
    Serial.println(theta_acc,   3);

    delay(LOOP_MS);
}

#else
  #error "MODE_RAW / MODE_CALIBRATION / MODE_KALMAN 중 하나를 정의하세요"
#endif