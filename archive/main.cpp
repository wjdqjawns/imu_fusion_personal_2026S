// /*
//   main.cpp — IMU Attitude Estimation Framework
//   Author : Beomjun Chung

//   ── Run Modes ────────────────────────────────────────────────────────────────
//   MODE_RAW
//       Binary frame stream for offline Allan variance / FFT analysis.
//       Baud: 921600.  Frame: [0xAA55, seq, t_ms, int16 ax/ay/az/gx/gy/gz/mx/my/mz]

//   MODE_CALIBRATION
//       Collect N still samples → print bias → paste into global_var.cpp.

//   MODE_STATE_ESTIMATION
//       All active filters run on the same sensor data each loop.
//       CSV: time_ms, ax,ay,az, gx_dps,gy_dps,gz_dps, mx,my,mz, [phi,theta,psi per filter]
//       Gyro input to filters converted to [rad/s] internally.

//   ── Filter selection ─────────────────────────────────────────────────────────
//   Define any combination in global_var.h:
//     FILTER_GYRO_ONLY  FILTER_EKF  FILTER_COMPLEMENTARY
//     FILTER_MAHONY     FILTER_MADGWICK
// */

// #include <Arduino.h>

// #include "global_var.h"
// #include "imu_sensor.h"
// #include "imu_calibration.h"
// #include "euler.h"
// #include "dcm.h"
// #include "transform.h"

// #if defined(FILTER_GYRO_ONLY)
//   #include "gyro_prop.h"
//   static GyroPropState s_gyro;
// #endif

// #if defined(FILTER_EKF)
//   #include "ekf.h"
//   static EkfState s_ekf;
// #endif

// #if defined(FILTER_COMPLEMENTARY)
//   #include "complementary.h"
//   static CfState s_cf;
// #endif

// #if defined(FILTER_MAHONY)
//   #include "mahony.h"
//   static MahonyState s_mahony;
// #endif

// #if defined(FILTER_MADGWICK)
//   #include "madgwick.h"
//   static MadgwickState s_madgwick;
// #endif

// static ImuSensor imu;

// static void printEulerDeg(const EulerAngle& e)
// {
//     Serial.print(e.phi   * RAD2DEG, 2); Serial.print(F(","));
//     Serial.print(e.theta * RAD2DEG, 2); Serial.print(F(","));
//     Serial.print(e.psi   * RAD2DEG, 2);
// }

// // =============================================================================
// #if defined(MODE_RAW)

// #pragma pack(push, 1)
// struct RawFrame {
//     uint16_t header;   // 0xAA55
//     uint16_t seq;
//     uint32_t t_ms;
//     int16_t ax, ay, az;
//     int16_t gx, gy, gz;
//     int16_t mx, my, mz;
// };
// #pragma pack(pop)

// static constexpr float SCALE_ACC  = 1000.0f;
// static constexpr float SCALE_GYRO = 1000.0f;
// static constexpr float SCALE_MAG  = 100.0f;
// static constexpr uint16_t FRAME_HDR = 0xAA55;
// static uint16_t seq = 0;

// void setup()
// {
//     Serial.begin(921600);
//     imu.begin(true);
// }

// void loop()
// {
//     static uint32_t last = 0;
//     uint32_t now = millis();
//     if (now - last < (uint32_t)LOOP_MS) return;
//     last += LOOP_MS;

//     imu.update();
//     g_imuRaw = imu.read();

//     RawFrame f;
//     f.header = FRAME_HDR;
//     f.seq    = seq++;
//     f.t_ms   = now;
//     f.ax = (int16_t)(g_imuRaw.ax * SCALE_ACC);
//     f.ay = (int16_t)(g_imuRaw.ay * SCALE_ACC);
//     f.az = (int16_t)(g_imuRaw.az * SCALE_ACC);
//     f.gx = (int16_t)(g_imuRaw.gx * SCALE_GYRO);
//     f.gy = (int16_t)(g_imuRaw.gy * SCALE_GYRO);
//     f.gz = (int16_t)(g_imuRaw.gz * SCALE_GYRO);
//     f.mx = (int16_t)(g_imuRaw.mx * SCALE_MAG);
//     f.my = (int16_t)(g_imuRaw.my * SCALE_MAG);
//     f.mz = (int16_t)(g_imuRaw.mz * SCALE_MAG);
//     Serial.write((uint8_t*)&f, sizeof(f));
// }

// // =============================================================================
// #elif defined(MODE_CALIBRATION)

// void setup()
// {
//     Serial.begin(SERIAL_BAUD);
//     imu.begin(false);
//     delay(1000);
//     Serial.println(F("=== IMU Calibration ==="));
//     Serial.println(F("Place flat and still. Starting in 3 s..."));
//     delay(3000);
//     ImuBias b = calibrate(imu, CALIB_N, CALIB_DLY);
//     printBias(b);
// }

// void loop() {}

// // =============================================================================
// #elif defined(MODE_STATE_ESTIMATION)

// void setup()
// {
//     Serial.begin(SERIAL_BAUD);
//     imu.begin(true);   // enable magnetometer
//     delay(1000);

//     // Seed filters from first IMU reading
//     imu.update();
//     g_imuRaw = imu.read();
//     g_imuCal = applyBias(g_imuRaw, g_bias);

//     float phi0, theta0, psi0;
//     accelRollPitch(g_imuCal.ax, g_imuCal.ay, g_imuCal.az, phi0, theta0);
//     psi0 = magYaw(g_imuCal.mx, g_imuCal.my, g_imuCal.mz, phi0, theta0);

// #if defined(FILTER_GYRO_ONLY)
//     gyroPropInit(s_gyro);
//     s_gyro.euler.phi   = phi0;
//     s_gyro.euler.theta = theta0;
//     s_gyro.euler.psi   = psi0;
// #endif

// #if defined(FILTER_EKF)
//     // Rotate initial mag reading to world frame to store as reference
//     float R_wb[9];
//     dcmFromEuler(phi0, theta0, psi0, R_wb);
//     float ref[3], bm[3] = { g_imuCal.mx, g_imuCal.my, g_imuCal.mz };
//     dcmRotate(R_wb, bm, ref);
//     ekfInit(s_ekf, phi0, theta0, psi0, ref[0], ref[1], ref[2]);
// #endif

// #if defined(FILTER_COMPLEMENTARY)
//     cfInit(s_cf, phi0, theta0, psi0);
// #endif

// #if defined(FILTER_MAHONY)
//     mahonyInit(s_mahony);
// #endif

// #if defined(FILTER_MADGWICK)
//     madgwickInit(s_madgwick);
// #endif

//     // CSV header
//     Serial.print(F("time_ms,ax,ay,az,gx_dps,gy_dps,gz_dps,mx,my,mz"));
// #if defined(FILTER_GYRO_ONLY)
//     Serial.print(F(",phi_gyro,theta_gyro,psi_gyro"));
// #endif
// #if defined(FILTER_EKF)
//     Serial.print(F(",phi_ekf,theta_ekf,psi_ekf"));
// #endif
// #if defined(FILTER_COMPLEMENTARY)
//     Serial.print(F(",phi_cf,theta_cf,psi_cf"));
// #endif
// #if defined(FILTER_MAHONY)
//     Serial.print(F(",phi_mahony,theta_mahony,psi_mahony"));
// #endif
// #if defined(FILTER_MADGWICK)
//     Serial.print(F(",phi_madgwick,theta_madgwick,psi_madgwick"));
// #endif
//     Serial.println();
// }

// void loop()
// {
//     static uint32_t last = 0;
//     uint32_t now = millis();
//     if (now - last < (uint32_t)LOOP_MS) return;
//     last += LOOP_MS;

//     imu.update();
//     g_imuRaw = imu.read();
//     g_imuCal = applyBias(g_imuRaw, g_bias);

//     const ImuData& d = g_imuCal;

//     // gyro in [rad/s] for filter input (sensor outputs [deg/s])
//     float gx = d.gx * DEG2RAD;
//     float gy = d.gy * DEG2RAD;
//     float gz = d.gz * DEG2RAD;

//     EulerAngle e;

//     // Predict
// #if defined(FILTER_GYRO_ONLY)
//     gyroPropUpdate(s_gyro, gx, gy, gz, Ts);
// #endif
// #if defined(FILTER_EKF)
//     ekfPredict(s_ekf, gx, gy, gz, Ts);
// #endif

//     // Update
// #if defined(FILTER_EKF)
//     ekfUpdateAccel(s_ekf, d.ax, d.ay, d.az);
//     ekfUpdateMag(s_ekf,   d.mx, d.my, d.mz);
// #endif

//     // Output
//     Serial.print(now);
//     Serial.print(F(","));
//     Serial.print(d.ax, 4); Serial.print(F(","));
//     Serial.print(d.ay, 4); Serial.print(F(","));
//     Serial.print(d.az, 4); Serial.print(F(","));
//     Serial.print(d.gx, 4); Serial.print(F(","));   // [deg/s] as-is
//     Serial.print(d.gy, 4); Serial.print(F(","));
//     Serial.print(d.gz, 4); Serial.print(F(","));
//     Serial.print(d.mx, 2); Serial.print(F(","));
//     Serial.print(d.my, 2); Serial.print(F(","));
//     Serial.print(d.mz, 2);

// #if defined(FILTER_GYRO_ONLY)
//     e = gyroPropGetEulerFromQuat(s_gyro);
//     Serial.print(F(",")); printEulerDeg(e);
// #endif
// #if defined(FILTER_EKF)
//     e = ekfGetEuler(s_ekf);
//     Serial.print(F(",")); printEulerDeg(e);
// #endif
// #if defined(FILTER_COMPLEMENTARY)
//     cfUpdate(s_cf, gx, gy, gz, d.ax, d.ay, d.az, Ts, CF_ALPHA);
//     e = cfGetEuler(s_cf);
//     Serial.print(F(",")); printEulerDeg(e);
// #endif
// #if defined(FILTER_MAHONY)
//     mahonyUpdate(s_mahony, d.ax, d.ay, d.az, gx, gy, gz, Ts, MAHONY_KP, MAHONY_KI);
//     e = mahonyGetEuler(s_mahony);
//     Serial.print(F(",")); printEulerDeg(e);
// #endif
// #if defined(FILTER_MADGWICK)
//     madgwickUpdate(s_madgwick, d.ax, d.ay, d.az, gx, gy, gz, Ts, MADGWICK_BETA);
//     e = madgwickGetEuler(s_madgwick);
//     Serial.print(F(",")); printEulerDeg(e);
// #endif

//     Serial.println();
// }

// #else
//   #error "Define exactly one of: MODE_RAW, MODE_CALIBRATION, MODE_STATE_ESTIMATION"
// #endif
