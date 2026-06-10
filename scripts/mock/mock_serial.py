"""
File Name: mock_serial.py
Author: beomjun chung
Update Date: 2026-06-02

Description:
    Arduino 없이 두 가지 인터페이스로 IMU 데이터 시뮬레이션.
    - MockSerial       - CSV 텍스트 readline() 인터페이스  (serial_logger.py용)
    - MockSerialBinary - 바이너리 프레임 read() 인터페이스 (data_logger.py / jitter_monitor.py용)
"""

import random
import struct
import time

# =======================================================================
# 1. CSV 텍스트 버전 (기존 serial_logger.py 호환)
# =======================================================================
class MockSerial:
    """100Hz CSV 텍스트 스트림. serial_logger.py의 readline() 인터페이스 전용."""

    def __init__(self, *args, **kwargs):
        self._t0 = time.time()

    def readline(self) -> bytes:
        time.sleep(0.01)
        t_ms = int((time.time() - self._t0) * 1000)

        # raw sensor data with noise
        ax = round(random.gauss(0, 0.1), 4)
        ay = round(random.gauss(0, 0.1), 4)
        az = round(random.gauss(9.8, 0.05), 4)
        gx = round(random.gauss(0, 0.5), 4)
        gy = round(random.gauss(0, 0.5), 4)
        gz = round(random.gauss(0, 0.5), 4)
        mx = round(random.gauss(30, 1), 4)
        my = round(random.gauss(-10, 1), 4)
        mz = round(random.gauss(40, 1), 4)

        # gyro propagation
        roll_gyro_prop  = round(random.gauss(0, 1), 4)
        pitch_gyro_prop = round(random.gauss(0, 1), 4)
        yaw_gyro_prop   = round(random.gauss(0, 1), 4)

        # complementary filter
        roll_cf  = round(random.gauss(0, 1), 4)
        pitch_cf = round(random.gauss(0, 1), 4)
        yaw_cf   = round(random.gauss(0, 1), 4)

        # extended kalman filter
        roll_ekf  = round(random.gauss(0, 1), 4)
        pitch_ekf = round(random.gauss(0, 1), 4)
        yaw_ekf   = round(random.gauss(0, 1), 4)

        # madgwick filter
        roll_madgwick  = round(random.gauss(0, 1), 4)
        pitch_madgwick = round(random.gauss(0, 1), 4)
        yaw_madgwick   = round(random.gauss(0, 1), 4)

        # mahony filter
        roll_mahony  = round(random.gauss(0, 1), 4)
        pitch_mahony = round(random.gauss(0, 1), 4)
        yaw_mahony   = round(random.gauss(0, 1), 4)

        line = (f"{t_ms},"
                f"{ax},{ay},{az},"
                f"{gx},{gy},{gz},"
                f"{mx},{my},{mz},"
                f"{roll_gyro_prop},{pitch_gyro_prop},{yaw_gyro_prop},"
                f"{roll_cf},{pitch_cf},{yaw_cf},"
                f"{roll_ekf},{pitch_ekf},{yaw_ekf},"
                f"{roll_madgwick},{pitch_madgwick},{yaw_madgwick},"
                f"{roll_mahony},{pitch_mahony},{yaw_mahony}\n")
        return line.encode("utf-8")

    def close(self):
        pass

# =======================================================================
# 2. 바이너리 프레임 버전 (data_logger.py / jitter_monitor.py 호환)
# =======================================================================
_FRAME_HEADER    = 0xAA55
_MSG_PAYLOAD_LEN = 102          # seq(2)+time_us(4)+24floats(96)

def _crc16(data: bytes) -> int:
    """CRC16-CCITT: init=0xFFFF, poly=0x1021"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def _build_frame(seq: int, t_us: int,
                 ax, ay, az,
                 gx, gy, gz,
                 mx, my, mz,
                 roll_gyro=0.0,  pitch_gyro=0.0,  yaw_gyro=0.0,
                 roll_cf=0.0,    pitch_cf=0.0,    yaw_cf=0.0,
                 roll_ekf=0.0,   pitch_ekf=0.0,   yaw_ekf=0.0,
                 roll_madgwick=0.0, pitch_madgwick=0.0, yaw_madgwick=0.0,
                 roll_mahony=0.0,   pitch_mahony=0.0,   yaw_mahony=0.0) -> bytes:

    header  = struct.pack('<HHH', _FRAME_HEADER, _MSG_PAYLOAD_LEN, seq)
    payload = struct.pack('<Iffffffffffffffffffffffff',
                          t_us,
                          ax, ay, az, gx, gy, gz, mx, my, mz,
                          roll_gyro, pitch_gyro, yaw_gyro,
                          roll_cf, pitch_cf, yaw_cf,
                          roll_ekf, pitch_ekf, yaw_ekf,
                          roll_madgwick, pitch_madgwick, yaw_madgwick,
                          roll_mahony, pitch_mahony, yaw_mahony)
    
    body    = header + payload

    return body + struct.pack('<H', _crc16(body))

class MockSerialBinary:
    """100 Hz 바이너리 프레임 스트림.
    data_logger.py / jitter_monitor.py 의 ser.read(ser.in_waiting or 1) 인터페이스 호환.

    jitter simulation: time_us 에 가우시안 노이즈(±JITTER_US_SIGMA) 추가.
    """
    TARGET_HZ       = 100.0
    JITTER_US_SIGMA = 150.0   # [us] 실 타이머 인터럽트 오차 모사

    def __init__(self, *args, **kwargs):
        self._t0     = time.perf_counter()
        self._seq    = 0
        self._buf    = bytearray()
        self._next_t = self._t0   # 다음 프레임이 만들어져야 하는 시각

    # --- serial.Serial interface ----------------------------
    @property
    def in_waiting(self) -> int:
        self._flush_pending()
        return len(self._buf)

    def read(self, n: int = 1) -> bytes:
        self._flush_pending()
        if len(self._buf) == 0:
            # 다음 프레임까지 짧게 대기
            sleep_s = max(0.0, self._next_t - time.perf_counter())
            time.sleep(min(sleep_s, 0.005))
            self._flush_pending()
        chunk = bytes(self._buf[:n])
        del self._buf[:n]
        return chunk

    def close(self):
        pass

    # --- make packet --------------------------------------------
    def _flush_pending(self):
        now = time.perf_counter()
        while self._next_t <= now:
            t_us = int((self._next_t - self._t0) * 1_000_000)
            t_us += int(random.gauss(0, self.JITTER_US_SIGMA))
            t_us = max(0, t_us)

            self._buf += _build_frame(
                self._seq, t_us,
                random.gauss(0.0,   0.05),   # ax             [g]
                random.gauss(0.0,   0.05),   # ay             [g]
                random.gauss(-1.0,  0.02),   # az             [g]
                random.gauss(0.0,   0.5),    # gx             [deg/s]
                random.gauss(0.0,   0.5),    # gy             [deg/s]
                random.gauss(0.0,   0.5),    # gz             [deg/s]
                random.gauss(30.0,  1.0),    # mx             [uT]
                random.gauss(-10.0, 1.0),    # my             [uT]
                random.gauss(40.0,  1.0),    # mz             [uT]
                random.gauss(0.0,   1.0),    # roll_gyro      [deg]
                random.gauss(0.0,   1.0),    # pitch_gyro     [deg]
                random.gauss(0.0,   1.0),    # yaw_gyro       [deg]
                random.gauss(0.0,   1.0),    # roll_cf        [deg]
                random.gauss(0.0,   1.0),    # pitch_cf       [deg]
                random.gauss(0.0,   1.0),    # yaw_cf         [deg]
                random.gauss(0.0,   1.0),    # roll_ekf       [deg]
                random.gauss(0.0,   1.0),    # pitch_ekf      [deg]
                random.gauss(0.0,   1.0),    # yaw_ekf        [deg]
                random.gauss(0.0,   1.0),    # roll_madgwick  [deg]
                random.gauss(0.0,   1.0),    # pitch_madgwick [deg]
                random.gauss(0.0,   1.0),    # yaw_madgwick   [deg]
                random.gauss(0.0,   1.0),    # roll_mahony    [deg]
                random.gauss(0.0,   1.0),    # pitch_mahony   [deg]
                random.gauss(0.0,   1.0),    # yaw_mahony     [deg]
            )
            self._seq  = (self._seq + 1) & 0xFFFF
            self._next_t += 1.0 / self.TARGET_HZ