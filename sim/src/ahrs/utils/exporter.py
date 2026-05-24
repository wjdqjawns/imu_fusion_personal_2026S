"""
File Name: ./src/ahrs/utils/exporter.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Export context and raw data (CSV) saving for AHRS simulation.

    ExportContext  — typed container for all output directory paths
    build_export   — create directory tree from config, return ExportContext
    save_*_csv     — write raw sensor / estimation data to CSV files

    Note on config key:
        Reads from cfg["output"] to match config.yaml's 'output:' section.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping

import numpy as np

_DEFAULT_SUBDIRS: dict[str, str] = {
    "data_dir": "data",
    "fig_dir": "fig",
    "log_dir": "log",
    "report_dir": "report",
}

@dataclass
class ExportContext:
    export_dir: Path
    data_dir: Path
    fig_dir: Path
    log_dir: Path
    report_dir: Path

    @classmethod
    def from_export_dir(
        cls,
        export_dir: Path,
        subdirs: Mapping[str, str] | None = None,
    ) -> "ExportContext":
        subdir_map = dict(_DEFAULT_SUBDIRS)
        if subdirs is not None:
            subdir_map.update(subdirs)

        paths: dict[str, Path] = {"export_dir": export_dir}
        for attr_name, folder_name in subdir_map.items():
            path = export_dir / folder_name
            path.mkdir(parents=True, exist_ok=True)
            paths[attr_name] = path

        return cls(**paths)

# export setup
def build_export(cfg: dict) -> ExportContext:
    """
    Create the run output directory tree from config and return ExportContext.

    Config key: cfg["export"] (fallback: cfg["output"])
        root_dir       (str)  — root export folder, default "export"
        timestamp_dirs (bool) — append YYYYMMDD_HHMMSS subfolder, default true
    """
    out_cfg    = cfg.get("export")
    base_dir   = Path(out_cfg.get("root_dir", out_cfg.get("base_dir", "export")))
    use_ts     = out_cfg.get("timestamp_dirs", True)

    if use_ts:
        ts         = datetime.now().strftime("%Y%m%d_%H%M%S")
        export_dir = base_dir / ts
    else:
        export_dir = base_dir / "latest"

    subdirs = out_cfg.get("subdirs")
    return ExportContext.from_export_dir(export_dir, subdirs)

# data export functions
def save_noise_static_csv(
    data_dir: Path,
    t: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    mag: np.ndarray,
) -> None:
    """Save static noise characterization data (Allan / PSD source)."""
    with (data_dir / "noise_static.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s",
                    "gx_rad_s", "gy_rad_s", "gz_rad_s",
                    "ax_m_s2",  "ay_m_s2",  "az_m_s2",
                    "mx_uT",    "my_uT",    "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *gyro[k], *accel[k], *mag[k]])

def save_noise_sphere_csv(
    data_dir: Path,
    t: np.ndarray,
    mag: np.ndarray,
) -> None:
    """Save spherical trajectory magnetometer data (calibration source)."""
    with (data_dir / "noise_sphere.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "mx_uT", "my_uT", "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *mag[k]])

def save_estimation_csv(
    data_dir: Path,
    truth,                  # TruthData
    raw_gyro: np.ndarray,
    raw_accel: np.ndarray,
    raw_mag: np.ndarray,
    results: dict,
    quat_to_euler_fn,       # Transforms.quat_to_euler — avoids circular import
) -> None:
    """
    Save ground truth and per-filter estimation results as CSV.

    Args:
        quat_to_euler_fn: callable (q) → (roll, pitch, yaw) [rad]
    """
    t = truth.t

    # truth.csv
    with (data_dir / "truth.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s",
                    "roll_deg", "pitch_deg", "yaw_deg",
                    "gx_rad_s", "gy_rad_s",  "gz_rad_s",
                    "ax_m_s2",  "ay_m_s2",   "az_m_s2",
                    "mx_uT",    "my_uT",      "mz_uT"])
        for k in range(len(t)):
            rpy = np.rad2deg(quat_to_euler_fn(truth.q[k]))
            w.writerow([t[k], *rpy, *raw_gyro[k], *raw_accel[k], *raw_mag[k]])

    # {filter}.csv
    for name, res in results.items():
        with (data_dir / f"{name}.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s",
                        "roll_deg",     "pitch_deg",     "yaw_deg",
                        "err_roll_deg", "err_pitch_deg", "err_yaw_deg",
                        "geo_err_deg",
                        "bgx_rad_s",    "bgy_rad_s",     "bgz_rad_s"])
            for k in range(len(t)):
                rpy = np.rad2deg(quat_to_euler_fn(res["q"][k]))
                w.writerow([t[k], *rpy,
                             *res["euler_err_deg"][k],
                             res["geodesic_err_deg"][k],
                             *res["bias"][k]])