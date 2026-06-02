/*
  src/raw.cpp
  Author : Beomjun Chung
  Updated: 2026-06-02

  Description
    MODE and active filters are selected in include/global_var.h.
*/

#include <Arduino.h>

#include "global_var.h"
#include "imu_sensor.h"
#include "crc16.h"

static cImuSensor imu;
static uint32_t last_us = 0;
static uint16_t seq = 0;

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
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    imu.begin();
    setupTimer1();
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

    // payload
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

    // CRC: header부터 mz까지
    g_msg_packet.crc = crc16((uint8_t*)&g_msg_packet, sizeof(g_msg_packet) - sizeof(uint16_t));

    Serial.write((uint8_t*)&g_msg_packet, sizeof(g_msg_packet));
}
#elif defined(MODE_CALIBRATION)
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    imu.begin();
}

void loop()
{
    if (imu.update())
    {
        sImuData data = imu.read();
        Serial.print("ax: "); Serial.print(data.ax, 3);
        Serial.print(", ay: "); Serial.print(data.ay, 3);
        Serial.print(", az: "); Serial.print(data.az, 3);
        Serial.print(", gx: "); Serial.print(data.gx, 3);
        Serial.print(", gy: "); Serial.print(data.gy, 3);
        Serial.print(", gz: "); Serial.print(data.gz, 3);
        Serial.print(", mx: "); Serial.print(data.mx, 3);
        Serial.print(", my: "); Serial.print(data.my, 3);
        Serial.print(", mz: "); Serial.println(data.mz, 3);
    }
}
#elif defined(MODE_ESTIMATION)
void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
}

void loop()
{
}
#else
  #error "Define exactly one of: MODE_RAW, MODE_CALIBRATION, MODE_STATE_ESTIMATION"
#endif