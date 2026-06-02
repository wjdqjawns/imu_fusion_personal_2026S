# data_logger.py
# Run from project root: python scripts/serial/data_logger.py
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

from frame_parser import FrameParser

MODE_DATA   = 'MOCK'   # 'REAL' | 'MOCK'
SERIAL_PORT = 'COM3'
SERIAL_BAUD = 115200

# scripts/serial/data_logger.py -> parent.parent.parent = project root
EXPORT_ROOT = Path(__file__).parent.parent.parent / 'export'

CSV_HEADER = ['time_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz']

def main():
    timestamp = datetime.now().strftime('%Y_%m_%d_%H%M%S')
    data_dir = EXPORT_ROOT / timestamp / 'data'
    data_dir.mkdir(parents=True, exist_ok=True)

    csv_path = data_dir / f'imu_{timestamp}.csv'
    print(f'[Logger] Mode    : {MODE_DATA}')
    print(f'[Logger] Saving  : {csv_path}')
    print('[Logger] Ctrl+C to stop.\n')

    if MODE_DATA == 'MOCK':
        from mock_serial import MockSerialBinary
        ser = MockSerialBinary()
    else:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

    parser = FrameParser()

    start_time  = time.time()
    total_count = 0
    last_report = start_time

    def on_exit(sig, frame):
        elapsed = time.time() - start_time
        rate = total_count / elapsed if elapsed > 0 else 0.0
        print('\n[Logger] Stopped.')
        print(f'  Total packets : {total_count}')
        print(f'  Drops         : {parser.drop_count}')
        print(f'  CRC errors    : {parser.crc_errors}')
        print(f'  Elapsed       : {elapsed:.1f} s')
        print(f'  Effective rate: {rate:.1f} Hz')
        ser.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)

        while True:
            raw = ser.read(ser.in_waiting or 1)
            if not raw:
                continue

            for pkt in parser.feed(raw):
                total_count += 1
                writer.writerow([
                    pkt['time_us'],
                    pkt['ax'], pkt['ay'], pkt['az'],
                    pkt['gx'], pkt['gy'], pkt['gz'],
                    pkt['mx'], pkt['my'], pkt['mz'],
                ])

            now = time.time()
            if now - last_report >= 1.0:
                elapsed = now - start_time
                rate = total_count / elapsed if elapsed > 0 else 0.0
                print(
                    f'[{elapsed:6.1f}s]'
                    f'  pkt:{total_count:6d}'
                    f'  drop:{parser.drop_count:4d}'
                    f'  crc_err:{parser.crc_errors:4d}'
                    f'  rate:{rate:5.1f}Hz'
                )
                last_report = now


if __name__ == '__main__':
    main()
