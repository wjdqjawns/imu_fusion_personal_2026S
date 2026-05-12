import csv
from pathlib import Path
import numpy as np
from src.utils.quaternion import euler_from_quaternion
from src.filters.mahony import MahonyFilter
from src.filters.madgwick import MadgwickFilter
from src.filters.ekf import EkfAhrs


def build_filters(cfg):
    return {
        "mahony": MahonyFilter(**cfg["mahony"]),
        "madgwick": MadgwickFilter(**cfg["madgwick"]),
        "ekf": EkfAhrs(**cfg["ekf"]),
    }


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def run_filters(sim, cfg):
    filters = build_filters(cfg)
    results = {}
    dt = sim.t[1] - sim.t[0]

    for name, filt in filters.items():
        q_hist = np.zeros((len(sim.t), 4), dtype=float)
        euler_hist = np.zeros((len(sim.t), 3), dtype=float)
        bias_hist = np.zeros((len(sim.t), 3), dtype=float)
        for k in range(len(sim.t)):
            q, b = filt.update(sim.gyro[k], sim.accel[k], sim.mag[k], dt)
            q_hist[k] = q
            euler_hist[k] = euler_from_quaternion(q)
            bias_hist[k] = b
        err = wrap_angle(euler_hist - sim.euler_true)
        results[name] = {
            "q": q_hist,
            "euler": euler_hist,
            "bias": bias_hist,
            "error": err,
            "rmse_deg": np.rad2deg(np.sqrt(np.mean(err**2, axis=0))),
        }
    return results


def save_results(sim, results, out_dir: str | Path):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    truth_path = out_dir / "truth.csv"
    with truth_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t", "roll_true_deg", "pitch_true_deg", "yaw_true_deg",
            "gx_rad_s", "gy_rad_s", "gz_rad_s",
            "ax_g", "ay_g", "az_g", "mx", "my", "mz", "temp_c"
        ])
        for k, tt in enumerate(sim.t):
            writer.writerow([
                tt, *np.rad2deg(sim.euler_true[k]), *sim.gyro[k], *sim.accel[k], *sim.mag[k], sim.temp[k]
            ])

    for name, res in results.items():
        path = out_dir / f"{name}.csv"
        with path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t", "roll_deg", "pitch_deg", "yaw_deg",
                "err_roll_deg", "err_pitch_deg", "err_yaw_deg",
                "bgx", "bgy", "bgz"
            ])
            for k, tt in enumerate(sim.t):
                writer.writerow([
                    tt,
                    *np.rad2deg(res["euler"][k]),
                    *np.rad2deg(res["error"][k]),
                    *res["bias"][k],
                ])
