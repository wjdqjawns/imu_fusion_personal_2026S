"""
File Name: ./scripts/analysis_noise.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  오프라인 노이즈 분석 (저장된 CSV 파일 기반)

    Usage:
        # 특정 run 디렉토리 지정
        py scripts/analysis_noise.py --run-dir export/2026-05-18_155300

        # 가장 최근 run 자동 선택
        py scripts/analysis_noise.py

    Input  : <run-dir>/data/noise_static.csv
             <run-dir>/data/noise_sphere.csv
    Output : <run-dir>/fig/allan_*.png, psd_*.png, mag_calibration.png

    noise_static.csv 없음 → main에서 calibration.enabled: true로 먼저 실행할 것
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ahrs.core.evaluation.allan import AllanVariance
from ahrs.core.calibration.mag_cal import MagCalibrator
from ahrs.utils.plotter import Plotter


def _latest_run_dir(base: Path) -> Path:
    candidates = sorted(
        [d for d in base.iterdir()
         if d.is_dir() and (d / "data").is_dir()],
        key=lambda d: d.name,
    )
    if not candidates:
        raise FileNotFoundError(f"No run directories found under {base}")
    return candidates[-1]


def _load_csv(path: Path) -> tuple[list[str], np.ndarray]:
    with path.open("r") as f:
        header = f.readline().strip().split(",")
        data   = np.loadtxt(f, delimiter=",")
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return header, data


def main():
    parser = argparse.ArgumentParser(
        description="Offline IMU noise & calibration analysis from saved CSVs"
    )
    parser.add_argument(
        "--run-dir", default=None,
        help="run 디렉토리 경로 (기본: export/ 내 최신 run)",
    )
    parser.add_argument("--dpi",   type=int, default=150)
    parser.add_argument("--style", default="ieee", choices=["ieee", "nature"])
    parser.add_argument("--show",  action="store_true")
    args = parser.parse_args()

    # ── run 디렉토리 결정 ─────────────────────────────────────────────────────
    if args.run_dir:
        run_dir = Path(args.run_dir)
    else:
        run_dir = _latest_run_dir(ROOT / "export")
    print(f"Run dir: {run_dir}")

    data_dir = run_dir / "data"
    fig_dir  = run_dir / "fig"
    fig_dir.mkdir(parents=True, exist_ok=True)

    # ── dt 추정 ──────────────────────────────────────────────────────────────
    static_path = data_dir / "noise_static.csv"
    sphere_path = data_dir / "noise_sphere.csv"

    if not static_path.exists() and not sphere_path.exists():
        print("[!] noise_static.csv 와 noise_sphere.csv 가 없습니다.")
        print("    config.yaml 에서 calibration.enabled: true 로 설정 후 재실행하세요.")
        print("    py -m ahrs.main --config config/config.yaml")
        sys.exit(1)

    plotter = (Plotter.for_ieee(dpi=args.dpi) if args.style == "ieee"
               else Plotter.for_nature(dpi=args.dpi))
    fmt = plotter.fmt

    # ── Allan variance + PSD ────────────────────────────────────────────────
    if static_path.exists():
        print("[1/2] Loading noise_static.csv...")
        header, static_data = _load_csv(static_path)
        col = {c: i for i, c in enumerate(header)}

        t_s    = static_data[:, col["t"]]
        dt     = float(t_s[1] - t_s[0]) if len(t_s) > 1 else 0.01
        fs     = 1.0 / dt

        gyro_s  = static_data[:, [col["gx_rad_s"], col["gy_rad_s"], col["gz_rad_s"]]]
        accel_s = static_data[:, [col["ax_m_s2"],  col["ay_m_s2"],  col["az_m_s2"]]]
        mag_s   = static_data[:, [col["mx_uT"],    col["my_uT"],    col["mz_uT"]]]

        print(f"  {len(t_s)} samples @ {fs:.0f} Hz  ({t_s[-1]:.0f} s)")

        # Allan variance
        print("  Computing Allan variance (gyro)...")
        av_g = AllanVariance(gyro_s, dt)
        tau_g, adev_g = av_g.compute()
        params_gyro   = av_g.extract_params()
        plotter.allan_variance(tau_g, adev_g, axis_labels=["x", "y", "z"],
                               out_path=fig_dir / f"allan_gyro.{fmt}")

        print("  Computing Allan variance (accel)...")
        av_a = AllanVariance(accel_s, dt)
        tau_a, adev_a = av_a.compute()
        params_accel  = av_a.extract_params()
        plotter.allan_variance(tau_a, adev_a, axis_labels=["x", "y", "z"],
                               out_path=fig_dir / f"allan_accel.{fmt}")

        # PSD
        print("  Computing PSD (Welch)...")
        plotter.psd(gyro_s, fs, title="Gyroscope PSD",
                    axis_labels=["x", "y", "z"], unit_label="(rad/s)²/Hz",
                    out_path=fig_dir / f"psd_gyro.{fmt}")
        plotter.psd(accel_s, fs, title="Accelerometer PSD",
                    axis_labels=["x", "y", "z"], unit_label="(m/s²)²/Hz",
                    out_path=fig_dir / f"psd_accel.{fmt}")
        plotter.psd(mag_s, fs, title="Magnetometer PSD (static)",
                    axis_labels=["x", "y", "z"], unit_label="µT²/Hz",
                    out_path=fig_dir / f"psd_mag.{fmt}")

        print("\n=== Gyro Allan Variance ===")
        for ax in ["x", "y", "z"]:
            print(f"  {ax}: ARW={params_gyro.get(f'arw_deg_per_sqrth_{ax}',0):.4f} deg/sqrt(h)"
                  f"  BI={params_gyro.get(f'bi_deg_h_{ax}',0):.4f} deg/h"
                  f"  RRW={params_gyro.get(f'rrw_deg_h_sqrth_{ax}',0):.4f} deg/h/sqrt(h)")
        print("\n=== Accel Allan Variance (VRW / BI) ===")
        for ax in ["x", "y", "z"]:
            print(f"  {ax}: VRW={params_accel.get(f'arw_deg_per_sqrth_{ax}',0):.4f}"
                  f"  BI={params_accel.get(f'bi_deg_h_{ax}',0):.4f}")
    else:
        print("[skip] noise_static.csv not found, skipping Allan/PSD")

    # ── Mag hard/soft iron ────────────────────────────────────────────────────
    if sphere_path.exists():
        print("\n[2/2] Loading noise_sphere.csv...")
        header, sphere_data = _load_csv(sphere_path)
        col = {c: i for i, c in enumerate(header)}
        mag_sp = sphere_data[:, [col["mx_uT"], col["my_uT"], col["mz_uT"]]]
        print(f"  {len(mag_sp)} samples")

        print("  Fitting ellipsoid (hard/soft iron)...")
        cal = MagCalibrator(method="ellipsoid_fit")
        cal.fit(mag_sp)
        mag_sp_cal = np.array([cal.apply(m) for m in mag_sp])

        plotter.mag_sphere(mag_sp, mag_sp_cal,
                           out_path=fig_dir / f"mag_calibration.{fmt}")

        raw_r = np.linalg.norm(mag_sp,     axis=1)
        cal_r = np.linalg.norm(mag_sp_cal, axis=1)
        rep = cal.report()
        print(f"\n=== Mag Calibration ===")
        print(f"  Raw |m|: {raw_r.mean():.2f} +/- {raw_r.std():.3f} uT")
        print(f"  Cal |m|: {cal_r.mean():.2f} +/- {cal_r.std():.3f} uT")
        print(f"  Hard iron [uT]: {[f'{v:.3f}' for v in rep['hard_iron_uT']]}")
    else:
        print("[skip] noise_sphere.csv not found, skipping mag calibration")

    if args.show:
        import matplotlib.pyplot as plt
        plt.show()

    print(f"\nFigures saved: {fig_dir}")


if __name__ == "__main__":
    main()
