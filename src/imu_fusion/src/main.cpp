/*
  src/main.cpp
  Author : Beomjun Chung
  Updated: 2026-06-10

  Description
    MODE and active filters are selected in include/global_var.h.
*/

#include <Arduino.h>

#include "global_var.h"
#include "imu_sensor.h"
#include "imu_calibration.h"
#include "crc16.h"

static cImuSensor imu;
static uint16_t seq;

// Timer1 CTC: 10ms
void setupTimer1()
{
    cli();
    TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
    OCR1A  = 2499;                          // 16MHz / 64 / 2500 = 100Hz
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11) | (1 << CS10);   // 프리스케일러 64
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

volatile bool imu_ready = false;
ISR(TIMER1_COMPA_vect)
{
    imu_ready = true;
}

#if defined(MODE_RAW)
void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial)
    {
        delay(10);
    }

    imu.begin();
    setupTimer1();

    seq = 0;
}

void loop()
{
    if (!imu_ready) return;
    imu_ready = false;

    imu.update();
    g_imuRaw = imu.read();

    // header
    g_msg_packet.header  = FRAME_HEADER;
    g_msg_packet.length  = MSG_PAYLOAD_LEN;
    g_msg_packet.seq     = seq++;

    // payload — raw IMU
    g_msg_packet.time_us = micros();
    g_msg_packet.ax = g_imuRaw.ax;
    g_msg_packet.ay = g_imuRaw.ay;
    g_msg_packet.az = g_imuRaw.az;
    g_msg_packet.gx = g_imuRaw.gx;
    g_msg_packet.gy = g_imuRaw.gy;
    g_msg_packet.gz = g_imuRaw.gz;
    g_msg_packet.mx = g_imuRaw.mx;
    g_msg_packet.my = g_imuRaw.my;
    g_msg_packet.mz = g_imuRaw.mz;

    // payload — angles (not computed in RAW mode)
    g_msg_packet.roll_gyro    = 0.0f; g_msg_packet.pitch_gyro    = 0.0f; g_msg_packet.yaw_gyro    = 0.0f;
    g_msg_packet.roll_cf      = 0.0f; g_msg_packet.pitch_cf      = 0.0f; g_msg_packet.yaw_cf      = 0.0f;
    g_msg_packet.roll_ekf     = 0.0f; g_msg_packet.pitch_ekf     = 0.0f; g_msg_packet.yaw_ekf     = 0.0f;
    g_msg_packet.roll_madgwick= 0.0f; g_msg_packet.pitch_madgwick= 0.0f; g_msg_packet.yaw_madgwick= 0.0f;
    g_msg_packet.roll_mahony  = 0.0f; g_msg_packet.pitch_mahony  = 0.0f; g_msg_packet.yaw_mahony  = 0.0f;

    // CRC: header부터 mz까지
    g_msg_packet.crc = crc16((uint8_t*)&g_msg_packet, sizeof(g_msg_packet) - sizeof(uint16_t));

    Serial.write((uint8_t*)&g_msg_packet, sizeof(g_msg_packet));
}

#elif defined(MODE_CALIBRATION)
void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(10); }
    imu.begin();

    // Phase 1: gyro bias + accel flat-rest offset
    Serial.println(F("[CAL] Phase 1: Place sensor FLAT and STILL."));
    Serial.println(F("[CAL] Starting in 3 seconds..."));
    delay(3000);
    Serial.println(F("[CAL] Collecting 500 samples..."));

    sImuBias sb = calibrateStaticBias(imu, CALIB_N, CALIB_DLY);
    g_bias.ax = sb.ax; g_bias.ay = sb.ay; g_bias.az = sb.az;
    g_bias.gx = sb.gx; g_bias.gy = sb.gy; g_bias.gz = sb.gz;

    Serial.print(F("[CAL] Accel bias  ax=")); Serial.print(sb.ax,4);
    Serial.print(F(" ay=")); Serial.print(sb.ay,4);
    Serial.print(F(" az=")); Serial.println(sb.az,4);
    Serial.print(F("[CAL] Gyro  bias  gx=")); Serial.print(sb.gx,4);
    Serial.print(F(" gy=")); Serial.print(sb.gy,4);
    Serial.print(F(" gz=")); Serial.println(sb.gz,4);

    // Phase 2: magnetometer hard-iron offset
    Serial.println(F("[CAL] Phase 2: ROTATE sensor through ALL orientations."));
    Serial.println(F("[CAL] You have 30 seconds. Starting now..."));

    sImuBias mb = calibrateMagBias(imu, 30000);
    g_bias.mx = mb.mx; g_bias.my = mb.my; g_bias.mz = mb.mz;

    Serial.print(F("[CAL] Mag bias    mx=")); Serial.print(mb.mx,2);
    Serial.print(F(" my=")); Serial.print(mb.my,2);
    Serial.print(F(" mz=")); Serial.println(mb.mz,2);

    printBias(g_bias);
}

void loop() {}

#elif defined(MODE_STATE_ESTIMATION) || defined(MODE_STATE_ESTIMATION_BINARY)

#include "orientation.h"
#include "gyro_prop.h"
#include "complementary.h"
#include "ekf.h"
#include "madgwick.h"
#include "mahony.h"

static sGyroPropState g_gyro;
static sCfState       g_cf;
static sEkfState      g_ekf;
static sMadgwickState g_madgwick;
static MahonyState    g_mahony;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(10); }

    imu.begin();
    delay(100);

    // One reading for initial roll/pitch/yaw estimates
    imu.update();
    g_imuRaw = imu.read();
    sImuData d0 = applyBias(g_imuRaw, g_bias);

    float phi0, theta0;
    accelRollPitch(d0.ax, d0.ay, d0.az, phi0, theta0);
    float psi0 = magYaw(d0.mx, d0.my, d0.mz, phi0, theta0);

    gyroPropInit(g_gyro);
    cfInit(g_cf, phi0, theta0, psi0);
    ekfInit(g_ekf, phi0, theta0, psi0, d0.mx, d0.my, d0.mz);
    madgwickInit(g_madgwick);
    mahonyInit(g_mahony);

    setupTimer1();
    seq = 0;
}

void loop()
{
    if (!imu_ready) return;
    imu_ready = false;

    imu.update();
    g_imuRaw = imu.read();
    sImuData d = applyBias(g_imuRaw, g_bias);

    // gyro: sensor outputs deg/s → convert to rad/s for filters
    float gx_r = d.gx * DEG2RAD;
    float gy_r = d.gy * DEG2RAD;
    float gz_r = d.gz * DEG2RAD;

    // 1. Gyro propagation (pure integration, quaternion)
    gyroPropUpdate(g_gyro, gx_r, gy_r, gz_r, Ts);
    EulerAngle e_gyro = gyroPropGetEulerFromQuat(g_gyro);

    // 2. Complementary filter (accel roll/pitch + mag yaw)
    cfUpdate(g_cf, gx_r, gy_r, gz_r, d.ax, d.ay, d.az, Ts, CF_ALPHA);
    cfUpdateYaw(g_cf, d.mx, d.my, d.mz, CF_ALPHA);
    EulerAngle e_cf = cfGetEuler(g_cf);

    // 3. EKF (6-state: angles + gyro bias)
    ekfPredict(g_ekf, gx_r, gy_r, gz_r, Ts);
    ekfUpdateAccel(g_ekf, d.ax, d.ay, d.az);
    ekfUpdateMag(g_ekf, d.mx, d.my, d.mz);
    EulerAngle e_ekf = ekfGetEuler(g_ekf);

    // 4. Madgwick gradient descent
    madgwickUpdate(g_madgwick, d.ax, d.ay, d.az, gx_r, gy_r, gz_r, Ts, MADGWICK_BETA);
    EulerAngle e_mad = madgwickGetEuler(g_madgwick);

    // 5. Mahony nonlinear complementary
    mahonyUpdate(g_mahony, d.ax, d.ay, d.az, gx_r, gy_r, gz_r, Ts, MAHONY_KP, MAHONY_KI);
    EulerAngle e_mah = mahonyGetEuler(g_mahony);

#if defined(MODE_STATE_ESTIMATION)
    // CSV: t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,roll_gyro,...,yaw_mahony
    Serial.print((float)micros() / 1000.0f, 3);
    Serial.print(','); Serial.print(g_imuRaw.ax, 4);
    Serial.print(','); Serial.print(g_imuRaw.ay, 4);
    Serial.print(','); Serial.print(g_imuRaw.az, 4);
    Serial.print(','); Serial.print(g_imuRaw.gx, 4);
    Serial.print(','); Serial.print(g_imuRaw.gy, 4);
    Serial.print(','); Serial.print(g_imuRaw.gz, 4);
    Serial.print(','); Serial.print(g_imuRaw.mx, 4);
    Serial.print(','); Serial.print(g_imuRaw.my, 4);
    Serial.print(','); Serial.print(g_imuRaw.mz, 4);
    Serial.print(','); Serial.print(e_gyro.phi   * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_gyro.theta * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_gyro.psi   * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_cf.phi     * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_cf.theta   * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_cf.psi     * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_ekf.phi    * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_ekf.theta  * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_ekf.psi    * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mad.phi    * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mad.theta  * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mad.psi    * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mah.phi    * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mah.theta  * RAD2DEG, 4);
    Serial.print(','); Serial.print(e_mah.psi    * RAD2DEG, 4);
    Serial.println();

#else  // MODE_STATE_ESTIMATION_BINARY
    g_msg_packet.header  = FRAME_HEADER;
    g_msg_packet.length  = MSG_PAYLOAD_LEN;
    g_msg_packet.seq     = seq++;
    g_msg_packet.time_us = micros();
    g_msg_packet.ax = g_imuRaw.ax; g_msg_packet.ay = g_imuRaw.ay; g_msg_packet.az = g_imuRaw.az;
    g_msg_packet.gx = g_imuRaw.gx; g_msg_packet.gy = g_imuRaw.gy; g_msg_packet.gz = g_imuRaw.gz;
    g_msg_packet.mx = g_imuRaw.mx; g_msg_packet.my = g_imuRaw.my; g_msg_packet.mz = g_imuRaw.mz;
    g_msg_packet.roll_gyro      = e_gyro.phi   * RAD2DEG;
    g_msg_packet.pitch_gyro     = e_gyro.theta * RAD2DEG;
    g_msg_packet.yaw_gyro       = e_gyro.psi   * RAD2DEG;
    g_msg_packet.roll_cf        = e_cf.phi     * RAD2DEG;
    g_msg_packet.pitch_cf       = e_cf.theta   * RAD2DEG;
    g_msg_packet.yaw_cf         = e_cf.psi     * RAD2DEG;
    g_msg_packet.roll_ekf       = e_ekf.phi    * RAD2DEG;
    g_msg_packet.pitch_ekf      = e_ekf.theta  * RAD2DEG;
    g_msg_packet.yaw_ekf        = e_ekf.psi    * RAD2DEG;
    g_msg_packet.roll_madgwick  = e_mad.phi    * RAD2DEG;
    g_msg_packet.pitch_madgwick = e_mad.theta  * RAD2DEG;
    g_msg_packet.yaw_madgwick   = e_mad.psi    * RAD2DEG;
    g_msg_packet.roll_mahony    = e_mah.phi    * RAD2DEG;
    g_msg_packet.pitch_mahony   = e_mah.theta  * RAD2DEG;
    g_msg_packet.yaw_mahony     = e_mah.psi    * RAD2DEG;
    g_msg_packet.crc = crc16((uint8_t*)&g_msg_packet, sizeof(g_msg_packet) - sizeof(uint16_t));
    Serial.write((uint8_t*)&g_msg_packet, sizeof(g_msg_packet));
#endif
}

#else
  #error "Define exactly one of: MODE_RAW, MODE_CALIBRATION, MODE_STATE_ESTIMATION, MODE_STATE_ESTIMATION_BINARY"
#endif
