#!/usr/bin/env python3
"""Quick checker for Allan analysis and integrated gyro angles.

Usage:
    python scripts/check_allan.py --csv path/to/noise_static.csv

If --csv omitted, script searches common export/data locations.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
# Ensure package import works: add sim/src to path
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ahrs.core.evaluation.allan import AllanVarianceAnalysis
from ahrs.utils.transforms import Transforms


def find_default_csv() -> Path | None:
    # common locations: export/*/data/noise_static.csv, export/*/data/raw_time_series.csv, data/noise_static.csv
    base = ROOT
    for pattern in ["export/*/data/noise_static.csv", "export/*/data/raw_time_series.csv", "data/noise_static.csv", "data/raw_time_series.csv"]:
        for p in base.glob(pattern):
            return p
    return None


def load_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    t_list = []
    gx = []
    gy = []
    gz = []
    ax = []
    ay = []
    az = []
    mx = []
    my = []
    mz = []
    with path.open("r", newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        hdr = next(r)
        for row in r:
            vals = [float(x) for x in row]
            t_list.append(vals[0])
            gx.append(vals[1]); gy.append(vals[2]); gz.append(vals[3])
            ax.append(vals[4]); ay.append(vals[5]); az.append(vals[6])
            # optional mag
            if len(vals) >= 10:
                mx.append(vals[7]); my.append(vals[8]); mz.append(vals[9])

    t = np.array(t_list)
    gyro = np.vstack([gx, gy, gz]).T
    accel = np.vstack([ax, ay, az]).T
    mag = np.vstack([mx, my, mz]).T if mx else np.zeros((len(t), 3))
    return t, gyro, accel, mag


def downsample(arr: np.ndarray, factor: int) -> np.ndarray:
    if factor <= 1:
        return arr
    return arr[::factor]


def run_allan_and_save(t: np.ndarray, data: np.ndarray, dt: float, out_dir: Path, name: str, n_tau: int = 100):
    av = AllanVarianceAnalysis(data, dt, n_tau=n_tau)
    tau, adev = av.compute()
    out_png = out_dir / f"allan_{name}.png"
    out_csv = out_dir / f"allan_{name}.csv"

    # save csv
    if adev.ndim == 1:
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            f.write("tau_s,adev\n")
            for a, b in zip(tau, adev):
                f.write(f"{a},{b}\n")
    else:
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            f.write("tau_s,adev_x,adev_y,adev_z\n")
            for k in range(len(tau)):
                row = adev[k]
                f.write(f"{tau[k]},{row[0]},{row[1]},{row[2]}\n")

    # plot
    plt.figure(figsize=(6, 4))
    if adev.ndim == 1:
        plt.loglog(tau, adev, linewidth=1)
    else:
        labels = ["x", "y", "z"]
        for i in range(adev.shape[1]):
            plt.loglog(tau, adev[:, i], linewidth=0.9, label=labels[i])
        plt.legend()
    plt.xlabel("tau [s]")
    plt.ylabel("Allan Dev")
    plt.grid(True, which="both", alpha=0.3)
    plt.title(f"Allan - {name}")
    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close()

    print(f"Saved Allan {name} -> {out_png}, {out_csv}")
    return tau, adev


def integrate_quaternion(gyro_rad_s: np.ndarray, dt: float) -> np.ndarray:
    """Integrate body-frame gyro into body->world quaternion trajectory."""
    q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    q_series = np.zeros((len(gyro_rad_s), 4), dtype=float)
    for k, omega in enumerate(gyro_rad_s):
        dq = Transforms.rotvec_to_quat(omega * dt)
        q = Transforms.quat_normalize(Transforms.quat_mult(q, dq))
        q_series[k] = q
    return q_series


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", type=Path, default=None, help="path to raw_time_series.csv or noise_static.csv")
    p.add_argument("--out", type=Path, default=None, help="output directory")
    p.add_argument("--n-tau", type=int, default=200)
    p.add_argument("--downsample", type=int, default=1, help="decimation factor for Allan (1=no downsample)")
    args = p.parse_args()

    csv_path = args.csv or find_default_csv()
    if csv_path is None or not csv_path.exists():
        print("No CSV found. Please provide --csv path to noise_static.csv or raw_time_series.csv")
        sys.exit(1)

    t, gyro, accel, mag = load_csv(csv_path)
    dt = float(np.median(np.diff(t)))
    print(f"Loaded {csv_path} | samples={len(t)} dt={dt}")

    out_dir = args.out or csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # Preprocess: remove mean (static bias) for visualization; Allan class handles raw data, but removing mean improves clarity
    gyro_mean = gyro.mean(axis=0)
    accel_mean = accel.mean(axis=0)
    gyro_zero = gyro - gyro_mean
    accel_zero = accel - accel_mean

    # optional downsample
    if args.downsample > 1:
        ds = args.downsample
        t_ds = downsample(t, ds)
        gyro_ds = downsample(gyro_zero, ds)
        accel_ds = downsample(accel_zero, ds)
        dt_ds = dt * ds
    else:
        t_ds = t
        gyro_ds = gyro_zero
        accel_ds = accel_zero
        dt_ds = dt

    # Allan for gyro (units: rad/s)
    run_allan_and_save(t_ds, gyro_ds, dt_ds, out_dir, "gyro", n_tau=args.n_tau)
    # Allan for accel (units: m/s^2)
    run_allan_and_save(t_ds, accel_ds, dt_ds, out_dir, "accel", n_tau=args.n_tau)

    # Integrated angles (approx per-axis) — show drift from bias
    angle = np.cumsum(gyro_zero * dt, axis=0)  # rad
    angle_deg = np.rad2deg(angle)
    plt.figure(figsize=(6, 3))
    plt.plot(t, angle_deg[:, 0], label='gx_int')
    plt.plot(t, angle_deg[:, 1], label='gy_int')
    plt.plot(t, angle_deg[:, 2], label='gz_int')
    plt.xlabel('t [s]')
    plt.ylabel('angle [deg] (approx)')
    plt.legend()
    plt.grid(True)
    pth = out_dir / 'integrated_gyro_angles.png'
    plt.tight_layout()
    plt.savefig(pth, dpi=150)
    plt.close()
    print(f"Saved integrated angles -> {pth}")

    # Quaternion integration -> actual roll/pitch/yaw attitude
    q_series = integrate_quaternion(gyro_zero, dt)
    rpy = np.rad2deg(np.array([Transforms.quat_to_euler(q) for q in q_series]))
    plt.figure(figsize=(6, 3))
    plt.plot(t, rpy[:, 0], label='roll')
    plt.plot(t, rpy[:, 1], label='pitch')
    plt.plot(t, rpy[:, 2], label='yaw')
    plt.xlabel('t [s]')
    plt.ylabel('attitude [deg]')
    plt.legend()
    plt.grid(True)
    pth = out_dir / 'integrated_attitude_rpy.png'
    plt.tight_layout()
    plt.savefig(pth, dpi=150)
    plt.close()
    print(f"Saved integrated attitude -> {pth}")


if __name__ == '__main__':
    main()
