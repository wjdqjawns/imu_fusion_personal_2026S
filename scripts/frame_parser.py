# frame_parser.py
import struct
from typing import Optional
import numpy as np

FRAME_HEADER = 0xAA55
FRAME_FMT    = '<HHHIfffffffffH'
FRAME_SIZE   = struct.calcsize(FRAME_FMT)  # 48 bytes
# AVR little-endian: uint16_t 0xAA55 → wire bytes [0x55, 0xAA]
_SYNC = struct.pack('<H', FRAME_HEADER)    # b'\x55\xaa'

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
        self.drop_count  = 0
        self.crc_errors  = 0
        self.prev_seq    = None

    def feed(self, data: bytes) -> list[dict]:
        """바이트 입력 → 파싱된 패킷 리스트 반환"""
        self._buf += data
        packets = []

        while len(self._buf) >= FRAME_SIZE:
            # 헤더 탐색 (little-endian: 0xAA55 → [0x55, 0xAA])
            if self._buf[0] != _SYNC[0] or self._buf[1] != _SYNC[1]:
                self._buf.pop(0)
                continue

            frame = bytes(self._buf[:FRAME_SIZE])

            # CRC 검증
            calc = crc16(frame[:-2])
            header, length, seq, time_us, \
            ax, ay, az, \
            gx, gy, gz, \
            mx, my, mz, \
            crc = struct.unpack(FRAME_FMT, frame)

            if calc != crc:
                self.crc_errors += 1
                self._buf.pop(0)  # 한 칸 밀기
                continue

            # 드랍 감지
            if self.prev_seq is not None:
                diff = (seq - self.prev_seq) & 0xFFFF
                if diff != 1:
                    self.drop_count += diff - 1
                    print(f"[Parser] DROP {self.prev_seq}→{seq}, "
                          f"누락 {diff-1}개")
            self.prev_seq = seq

            self._buf = self._buf[FRAME_SIZE:]

            packets.append({
                'seq':     seq,
                'time_us': time_us,
                'ax': ax, 'ay': ay, 'az': az,
                'gx': gx, 'gy': gy, 'gz': gz,
                'mx': mx, 'my': my, 'mz': mz,
            })

        return packets