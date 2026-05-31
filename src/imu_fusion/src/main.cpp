/*
  main.cpp — IMU Attitude Estimation Framework
  Author : Beomjun Chung
  Updated: 2026-05-19

  MODE and active filters are selected in include/global_var.h.

  ── Run Modes ────────────────────────────────────────────────────────────────────
  MODE_RAW
      Raw sensor CSV for offline noise / Allan variance analysis.
      Header: time_ms,ax,ay,az,gx,gy,gz,mx,my,mz

  MODE_CALIBRATION
      Collect N still samples → print bias values → paste into global_var.cpp.

  MODE_STATE_ESTIMATION
      All active filters run simultaneously on the same sensor data each loop.
      Enables fair single-session comparison without re-running experiments.
      Header: time_ms,[gyro_only],[ekf],[cf],[mahony],[madgwick]
        phi_deg,theta_deg,psi_deg per filter (psi=0 for EKF, no mag)

  ── Filter selection ─────────────────────────────────────────────────────────
  Define any combination in global_var.h:
    #define FILTER_GYRO_ONLY
    #define FILTER_EKF
    #define FILTER_COMPLEMENTARY
    #define FILTER_MAHONY
    #define FILTER_MADGWICK
*/

#include <Arduino.h>

#include "global_var.h"
#include "imu_sensor.h"
#include "imu_calibration.h"
#include "euler.h"
#include "transform.h"

#if defined(FILTER_GYRO_ONLY)
  #include "gyro_prop.h"
  static GyroPropState s_gyro;
#endif

#if defined(FILTER_EKF)
  #include "kalman_filter.h"
  static EkfState s_ekf;
#endif

#if defined(FILTER_COMPLEMENTARY)
  #include "complementary.h"
  static CfState s_cf;
#endif

#if defined(FILTER_MAHONY)
  #include "mahony.h"
  static MahonyState s_mahony;
#endif

#if defined(FILTER_MADGWICK)
  #include "madgwick.h"
  static MadgwickState s_madgwick;
#endif

static ImuSensor imu;

// ─── Helper: print one Euler angle set (deg) ─────────────────────────────────
static void printEuler(const EulerAngle& e)
{
    Serial.print(e.phi   * RAD2DEG, 2); Serial.print(F(","));
    Serial.print(e.theta * RAD2DEG, 2); Serial.print(F(","));
    Serial.print(e.psi   * RAD2DEG, 2);
}

// =============================================================================
#if defined(MODE_RAW)

#pragma pack(push, 1)

// ===============================
// IMU data (scaled int)
// ===============================
struct ImuMeasurement
{
    uint32_t t_ms;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t mx;
    int16_t my;
    int16_t mz;
};

// ===============================
// Full frame with sync
// ===============================
typedef struct sFrame
{
    uint16_t header;   // 0xAA55
    uint16_t seq;
    sImuMeasurement data;
}sFrame;

#pragma pack(pop)

// ===============================
// Scaling
// ===============================
constexpr float SCALE_ACC  = 1000.0f;
constexpr float SCALE_GYRO = 1000.0f;
constexpr float SCALE_MAG  = 100.0f;

// ===============================
// constexpr uint16_t LOOP_MS = 10;
constexpr uint16_t HEADER  = 0xAA55;

uint16_t seq = 0;

void setup()
{
    Serial.begin(921600);   // 중요
    imu.begin(true);
}

// ===============================
void loop()
{
    static uint32_t last = 0;
    uint32_t now = millis();

    if (now - last < LOOP_MS) return;
    last += LOOP_MS;

    imu.update();
    g_imuRaw = imu.read();

    sFrame f;

    // header + seq
    f.header = HEADER;
    f.seq = seq++;

    // timestamp
    f.data.t_ms = now;

    // IMU scaling
    f.data.ax = (int16_t)(g_imuRaw.ax * SCALE_ACC);
    f.data.ay = (int16_t)(g_imuRaw.ay * SCALE_ACC);
    f.data.az = (int16_t)(g_imuRaw.az * SCALE_ACC);

    f.data.gx = (int16_t)(g_imuRaw.gx * SCALE_GYRO);
    f.data.gy = (int16_t)(g_imuRaw.gy * SCALE_GYRO);
    f.data.gz = (int16_t)(g_imuRaw.gz * SCALE_GYRO);

    f.data.mx = (int16_t)(g_imuRaw.mx * SCALE_MAG);
    f.data.my = (int16_t)(g_imuRaw.my * SCALE_MAG);
    f.data.mz = (int16_t)(g_imuRaw.mz * SCALE_MAG);

    // send full frame
    Serial.write((uint8_t*)&f, sizeof(f));
}

// =============================================================================
#elif defined(MODE_CALIBRATION)

void setup()
{
    Serial.begin(SERIAL_BAUD);
    imu.begin(false);
    delay(1000);
    Serial.println(F("=== IMU Calibration ==="));
    Serial.println(F("Place flat and still. Starting in 3 s..."));
    delay(3000);
    ImuBias b = calibrate(imu, CALIB_N, CALIB_DLY);
    printBias(b);
}

void loop() {}

// =============================================================================
#elif defined(MODE_STATE_ESTIMATION)

void setup()
{
    Serial.begin(SERIAL_BAUD);
    imu.begin(false);
    delay(1000);

    // Seed all filters from first accelerometer reading
    imu.update();
    g_imuRaw = imu.read();
    g_imuCal = applyBias(g_imuRaw, g_bias);

    float phi0, theta0;
    accelRollPitch(g_imuCal.ax, g_imuCal.ay, g_imuCal.az, phi0, theta0);

#if defined(FILTER_GYRO_ONLY)
    gyroPropInit(s_gyro);
    // Seed euler from accel so comparison starts from same origin
    s_gyro.euler.phi   = phi0;
    s_gyro.euler.theta = theta0;
#endif

#if defined(FILTER_EKF)
    ekfInit(s_ekf, phi0, theta0);
#endif

#if defined(FILTER_COMPLEMENTARY)
    cfInit(s_cf, phi0, theta0, 0.0f);
#endif

#if defined(FILTER_MAHONY)
    mahonyInit(s_mahony);
#endif

#if defined(FILTER_MADGWICK)
    madgwickInit(s_madgwick);
#endif

    // CSV header — one column group per active filter
    Serial.print(F("time_ms"));
#if defined(FILTER_GYRO_ONLY)
    Serial.print(F(",phi_gyro,theta_gyro,psi_gyro"));
#endif
#if defined(FILTER_EKF)
    Serial.print(F(",phi_ekf,theta_ekf,psi_ekf"));
#endif
#if defined(FILTER_COMPLEMENTARY)
    Serial.print(F(",phi_cf,theta_cf,psi_cf"));
#endif
#if defined(FILTER_MAHONY)
    Serial.print(F(",phi_mahony,theta_mahony,psi_mahony"));
#endif
#if defined(FILTER_MADGWICK)
    Serial.print(F(",phi_madgwick,theta_madgwick,psi_madgwick"));
#endif
    Serial.println();
}

void loop()
{
    imu.update();
    g_imuRaw = imu.read();
    g_imuCal = applyBias(g_imuRaw, g_bias);

    const ImuData& d = g_imuCal;
    EulerAngle e;

    // Serial.print(millis());

#if defined(FILTER_GYRO_ONLY)
    gyroPropUpdate(s_gyro, d.gx, d.gy, d.gz, Ts);
    e = gyroPropGetEulerFromQuat(s_gyro);   // quaternion output (no gimbal lock)
    Serial.print(F(",")); printEuler(e);
#endif

#if defined(FILTER_EKF)
    ekfUpdate(s_ekf, d.gx, d.gy, d.gz, d.ax, d.ay, d.az, Ts);
    e = ekfGetEuler(s_ekf);
    Serial.print(F(",")); printEuler(e);
#endif

#if defined(FILTER_COMPLEMENTARY)
    cfUpdate(s_cf, d.gx, d.gy, d.gz, d.ax, d.ay, d.az, Ts, CF_ALPHA);
    e = cfGetEuler(s_cf);
    Serial.print(F(",")); printEuler(e);
#endif

#if defined(FILTER_MAHONY)
    mahonyUpdate(s_mahony, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, Ts, MAHONY_KP, MAHONY_KI);
    e = mahonyGetEuler(s_mahony);
    Serial.print(F(",")); printEuler(e);
#endif

#if defined(FILTER_MADGWICK)
    madgwickUpdate(s_madgwick, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, Ts, MADGWICK_BETA);
    e = madgwickGetEuler(s_madgwick);
    Serial.print(F(",")); printEuler(e);
#endif

    Serial.println();
    delay(LOOP_MS);
}

#else
  #error "Define one of: MODE_RAW, MODE_CALIBRATION, MODE_STATE_ESTIMATION"
#endif
