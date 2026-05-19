from pathlib import Path
from datetime import datetime
import serial
import struct
import csv

# =========================================================
# Save path
# =========================================================
SAVE_DIR = Path(__file__).parent.parent / "data" / "raw_data"
SAVE_DIR.mkdir(parents=True, exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = SAVE_DIR / f"imu_log_{timestamp}.csv"

print("Saving to:", filename)

# =========================================================
# Serial
# =========================================================
ser = serial.Serial("COM6", 115200, timeout=1)

# struct format (little endian)
FMT = "<Ihhhhhhhhh"
SIZE = struct.calcsize(FMT)

# =========================================================
# CSV writer
# =========================================================
with open(filename, "w", newline="") as f:
    writer = csv.writer(f)

    # header
    writer.writerow([
        "t_ms",
        "ax","ay","az",
        "gx","gy","gz",
        "mx","my","mz"
    ])

    try:
        buffer = bytearray()

        while True:
            buffer += ser.read(ser.in_waiting or 1)

            while len(buffer) >= SIZE:
                packet = buffer[:SIZE]
                buffer = buffer[SIZE:]

                data = struct.unpack(FMT, packet)

                writer.writerow(data)

    except KeyboardInterrupt:
        print("Stopped")

    finally:
        ser.close()