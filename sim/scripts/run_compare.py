"""
File Name: ./scripts/run_compare.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  오프라인 필터 비교 플롯 생성 (저장된 CSV 파일 기반)

    Usage:
        # 특정 run 디렉토리 지정
        py scripts/run_compare.py --run-dir export/2026-05-18_155300

        # 가장 최근 run 자동 선택
        py scripts/run_compare.py

    Input  : <run-dir>/data/truth.csv, <run-dir>/data/{filter}.csv
    Output : <run-dir>/fig/attitude_comparison.png  (+ 5종 추가 플롯)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ahrs.utils.plotter import Plotter

# 필터 CSV 파일 이름 (truth.csv 제외)
_KNOWN_FILTERS = {"complementary", "mahony", "madgwick", "ekf"}


def _latest_run_dir(base: Path) -> Path:
    """export/ 하위에서 가장 최근 타임스탬프 run 디렉토리 반환."""
    candidates = sorted(
        [d for d in base.iterdir()
         if d.is_dir() and (d / "data" / "truth.csv").exists()],
        key=lambda d: d.name,
    )
    if not candidates:
        raise FileNotFoundError(f"No valid run directories found under {base}")
    return candidates[-1]


def _load_csv(path: Path) -> dict[str, np.ndarray]:
    """헤더 1행 + 나머지 float 데이터 로드."""
    with path.open("r") as f:
        header = f.readline().strip().split(",")
        data   = np.loadtxt(f, delimiter=",")
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return {col: data[:, i] for i, col in enumerate(header)}


def _collect_disturbance_spans(run_dir: Path) -> list[tuple[float, float]] | None:
    """config.yaml이 있으면 disturbance 구간 읽기, 없으면 None."""
    cfg_path = run_dir.parents[1] / "config" / "config.yaml"
    if not cfg_path.exists():
        return None
    try:
        import yaml
        with cfg_path.open("r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        dist_cfg = cfg.get("imu", {}).get("disturbance", {})
        spans = []
        for key in ("linear_acceleration", "magnetic"):
            sec = dist_cfg.get(key, {})
            if sec.get("enabled", False):
                for ev in sec.get("events", []):
                    t0 = float(ev["onset_time_s"])
                    spans.append((t0, t0 + float(ev["duration_s"])))
        return spans or None
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Offline filter comparison plots from saved CSVs"
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

    # ── truth.csv 로드 ────────────────────────────────────────────────────────
    truth_csv  = _load_csv(data_dir / "truth.csv")
    t          = truth_csv["t"]
    truth_euler = np.column_stack([
        truth_csv["roll_deg"],
        truth_csv["pitch_deg"],
        truth_csv["yaw_deg"],
    ])

    # ── filter CSV 로드 ───────────────────────────────────────────────────────
    filter_euler:  dict[str, np.ndarray] = {}
    filter_errors: dict[str, np.ndarray] = {}
    geo_errors:    dict[str, np.ndarray] = {}
    rmse_table:    dict[str, dict]       = {}

    for csv_path in sorted(data_dir.glob("*.csv")):
        name = csv_path.stem
        if name == "truth" or name.startswith("noise"):
            continue
        d = _load_csv(csv_path)
        filter_euler[name]  = np.column_stack([d["roll_deg"], d["pitch_deg"], d["yaw_deg"]])
        filter_errors[name] = np.column_stack([
            d["err_roll_deg"], d["err_pitch_deg"], d["err_yaw_deg"]
        ])
        geo_errors[name] = d["geo_err_deg"]

        err = filter_errors[name]
        geo = geo_errors[name]
        # wrap [-180, 180]
        err_w = (err + 180) % 360 - 180
        rmse_table[name] = {
            "roll":     float(np.sqrt(np.mean(err_w[:, 0]**2))),
            "pitch":    float(np.sqrt(np.mean(err_w[:, 1]**2))),
            "yaw":      float(np.sqrt(np.mean(err_w[:, 2]**2))),
            "geodesic": float(np.sqrt(np.mean(geo**2))),
        }

    if not filter_euler:
        print("No filter CSVs found. Run `py -m ahrs.main` first.")
        return

    # ── disturbance spans ─────────────────────────────────────────────────────
    dist_spans = _collect_disturbance_spans(run_dir)

    # ── plots ─────────────────────────────────────────────────────────────────
    plotter = (Plotter.for_ieee(dpi=args.dpi) if args.style == "ieee"
               else Plotter.for_nature(dpi=args.dpi))
    fmt = plotter.fmt

    plotter.attitude_comparison(t, truth_euler, filter_euler,
                                out_path=fig_dir / f"attitude_comparison.{fmt}",
                                disturbance_spans=dist_spans)
    plotter.attitude_error(t, filter_errors,
                           out_path=fig_dir / f"attitude_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.geodesic_error(t, geo_errors,
                           out_path=fig_dir / f"geodesic_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.rmse_bar(rmse_table, out_path=fig_dir / f"rmse_bar.{fmt}")
    plotter.trajectory_3d(truth_euler, filter_euler,
                          out_path=fig_dir / f"trajectory_3d.{fmt}")
    plotter.convergence(t, geo_errors, out_path=fig_dir / f"convergence.{fmt}")

    if args.show:
        import matplotlib.pyplot as plt
        plt.show()

    # ── RMSE 요약 ──────────────────────────────────────────────────────────────
    print(f"Figures saved: {fig_dir}")
    print(f"\n{'Filter':<15} {'Roll':>8} {'Pitch':>8} {'Yaw':>8} {'Geodesic':>10}  [RMSE deg]")
    print("-" * 55)
    for name, row in rmse_table.items():
        print(f"{name:<15} {row['roll']:>8.3f} {row['pitch']:>8.3f} "
              f"{row['yaw']:>8.3f} {row['geodesic']:>10.3f}")


if __name__ == "__main__":
    main()
