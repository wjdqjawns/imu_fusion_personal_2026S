# frame_parser.py
import struct
from typing import Optional
import numpy as np

FRAME_HEADER = 0xAA55
# layout: header(H) length(H) seq(H) time_us(I)
#         ax ay az gx gy gz mx my mz              (9f raw)
#         roll_gyro  pitch_gyro  yaw_gyro          (3f)
#         roll_cf    pitch_cf    yaw_cf             (3f)
#         roll_ekf   pitch_ekf   yaw_ekf            (3f)
#         roll_madgwick pitch_madgwick yaw_madgwick  (3f)
#         roll_mahony   pitch_mahony   yaw_mahony    (3f)
#         crc(H)
# total: 2+2+2+4 + 24*4 + 2 = 108 bytes
FRAME_FMT  = '<HHHIffffffffffffffffffffffffH'
FRAME_SIZE = struct.calcsize(FRAME_FMT)  # 108 bytes
# AVR little-endian: uint16_t 0xAA55 → wire bytes [0x55, 0xAA]
_SYNC = struct.pack('<H', FRAME_HEADER)  # b'\x55\xaa'

_FIELDS = (
    'header', 'length', 'seq', 'time_us',
    'ax', 'ay', 'az',
    'gx', 'gy', 'gz',
    'mx', 'my', 'mz',
    'roll_gyro',     'pitch_gyro',     'yaw_gyro',
    'roll_cf',       'pitch_cf',       'yaw_cf',
    'roll_ekf',      'pitch_ekf',      'yaw_ekf',
    'roll_madgwick', 'pitch_madgwick', 'yaw_madgwick',
    'roll_mahony',   'pitch_mahony',   'yaw_mahony',
    'crc',
)

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF \
                  if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

class FrameParser:
    def __init__(self):
        self._buf = bytearray()
        self.drop_count = 0
        self.crc_errors = 0
        self.prev_seq   = None

    def feed(self, data: bytes) -> list[dict]:
        """Feed bytes into the parser and return a list of parsed packets."""
        self._buf += data
        packets = []

        while len(self._buf) >= FRAME_SIZE:
            # find header (little-endian: 0xAA55 → [0x55, 0xAA])
            if self._buf[0] != _SYNC[0] or self._buf[1] != _SYNC[1]:
                self._buf.pop(0)
                continue

            frame = bytes(self._buf[:FRAME_SIZE])

            # validate CRC (covers header through last angle field)
            calc = crc16(frame[:-2])
            values = struct.unpack(FRAME_FMT, frame)
            parsed = dict(zip(_FIELDS, values))

            if calc != parsed['crc']:
                self.crc_errors += 1
                self._buf.pop(0)
                continue

            # detect packet drop
            seq = parsed['seq']
            if self.prev_seq is not None:
                diff = (seq - self.prev_seq) & 0xFFFF
                if diff != 1:
                    self.drop_count += diff - 1
                    print(f"[Parser] DROP {self.prev_seq}→{seq}, "
                          f"missing {diff - 1} packets")
            self.prev_seq = seq

            self._buf = self._buf[FRAME_SIZE:]

            # strip framing fields before returning
            del parsed['header']
            del parsed['length']
            del parsed['crc']
            packets.append(parsed)

        return packets
