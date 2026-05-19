from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# CSV 파일 경로
# =========================================================

csv_path = Path("./data/raw_data/imu_log_20260519_050053.csv")

# =========================================================
# CSV 읽기
# =========================================================

df = pd.read_csv(csv_path)

# 시간 [ms] -> [s]
t_ms = df["time_ms"].to_numpy()
t = t_ms * 1e-3

# sampling frequency 계산
dt = np.mean(np.diff(t))
fs = 1.0 / dt

print(f"Sampling Frequency: {fs:.2f} Hz")

# =========================================================
# Time Domain Plot
# =========================================================

plt.figure(figsize=(12, 6))

for col in df.columns:
    if col == "time_ms":
        continue

    plt.plot(t, df[col], label=col)

plt.xlabel("Time [s]")
plt.ylabel("IMU Data")
plt.title("IMU Raw Data")
plt.legend()
plt.grid(True)
plt.savefig("imu_time_domain.png", dpi=300)
# =========================================================
# FFT Plot
# =========================================================

plt.figure(figsize=(12, 6))

N = len(t)

for col in df.columns:

    if col == "time_ms":
        continue

    x = df[col].to_numpy()

    # FFT
    X = np.fft.rfft(x)

    # Frequency axis
    freq = np.fft.rfftfreq(N, d=dt)

    # Magnitude
    mag = np.abs(X) / N

    plt.plot(freq, mag, label=col)

plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.title("FFT")
plt.legend()
plt.grid(True)
plt.savefig("imu_fft.png", dpi=300)
# =========================================================
# PSD Plot
# =========================================================

plt.figure(figsize=(12, 6))

for col in df.columns:

    if col == "time_ms":
        continue

    x = df[col].to_numpy()

    # FFT
    X = np.fft.rfft(x)

    freq = np.fft.rfftfreq(N, d=dt)

    # PSD
    psd = (1.0 / (fs * N)) * np.abs(X) ** 2

    plt.semilogy(freq, psd, label=col)

plt.xlabel("Frequency [Hz]")
plt.ylabel("PSD")
plt.title("Power Spectral Density")
plt.legend()
plt.grid(True)
plt.savefig("imu_psd.png", dpi=300)
# =========================================================
# Show
# =========================================================

plt.show()