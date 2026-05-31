"""
File Name: ./src/ahrs/pipeline/stage/stage_report.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 7 — Plot + Report

    Stage 5, 6의 결과를 받아 모든 플롯을 저장하고
    Markdown / JSON / HTML 리포트를 생성.

    입력:
        cfg         : config dict
        char_results: Stage 5 출력 dict  {"noise": ..., "calibration": ...}
        est_results : Stage 6 출력 dict  {"truth": ..., "filters": ..., ...}
        ctx         : ExportContext
        plotter     : Plotter

    출력: 없음 (side effect only)
        fig/characterization/allan_gyro.png, allan_accel.png
        fig/characterization/psd_*.png, fft_*.png
        fig/estimation/attitude_comparison.png, attitude_error.png, ...
        report/report_{ts}.md, report_{ts}.json, report_{ts}.html
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from ahrs.utils.transforms import Transforms
from ahrs.utils.reporter import FilterReport

logger = logging.getLogger("ahrs_sim.report")


def run_report(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    ctx,
    plotter,
) -> None:
    """Stage 7 전체 실행."""
    fig_char = ctx.fig_dir / "characterization"
    fig_est  = ctx.fig_dir / "estimation"
    for d in (fig_char, fig_est, ctx.report_dir):
        d.mkdir(parents=True, exist_ok=True)

    fmt = plotter.fmt

    _plot_characterization(char_results, fig_char, plotter, fmt)
    _plot_estimation(cfg, est_results, fig_est, plotter, fmt)
    _save_report(cfg, char_results, est_results, ctx.report_dir)

    logger.info("[Stage7] Report saved | %s", ctx.report_dir)


# ── Characterization 플롯 ─────────────────────────────────────────────────────
def _plot_characterization(
    char_results: dict,
    fig_dir: Path,
    plotter,
    fmt: str,
) -> None:
    noise = char_results.get("noise", {})

    if "allan_gyro" in noise:
        r = noise["allan_gyro"]
        plotter.allan_variance( r["tau"], r["adev"], axis_labels=["x", "y", "z"], out_path=fig_dir / f"allan_gyro.{fmt}")

    if "allan_accel" in noise:
        r = noise["allan_accel"]
        plotter.allan_variance(
            r["tau"], r["adev"], axis_labels=["x", "y", "z"],
            out_path=fig_dir / f"allan_accel.{fmt}"
        )

    for sensor_name in ("gyro", "accel", "mag"):
        key = f"psd_{sensor_name}"
        if key in noise:
            r = noise[key]
            plotter.psd(
                r["power"], fs=None, freq=r["freq"],
                title=f"{sensor_name.capitalize()} PSD",
                axis_labels=["x", "y", "z"],
                out_path=fig_dir / f"psd_{sensor_name}.{fmt}"
            )

    for sensor_name in ("gyro", "accel", "mag"):
        key = f"fft_{sensor_name}"
        if key in noise:
            r = noise[key]
            plotter.fft_spectrum(
                r["amp"], freq=r["freq"],
                title=f"{sensor_name.capitalize()} FFT",
                axis_labels=["x", "y", "z"],
                out_path=fig_dir / f"fft_{sensor_name}.{fmt}"
            )

def _plot_estimation(
    cfg: dict,
    est_results: dict,
    fig_dir: Path,
    plotter,
    fmt: str,
) -> None:
    truth   = est_results["truth"]
    filters = est_results["filters"]

    truth_euler = np.rad2deg(np.array(
        [Transforms.quat_to_euler(q) for q in truth.q]
    ))
    filter_euler  = {name: res["euler_deg"]     for name, res in filters.items()}
    filter_errors = {name: res["euler_err_deg"] for name, res in filters.items()}
    geo_errors    = {name: res["geodesic_err"]  for name, res in filters.items()}
    rmse_table    = {
        name: {
            "roll":     res["rmse_deg"][0],
            "pitch":    res["rmse_deg"][1],
            "yaw":      res["rmse_deg"][2],
            "geodesic": res["geodesic_rmse"],
        }
        for name, res in filters.items()
    }

    dist_spans = _collect_disturbance_spans(cfg)
    traj_type  = cfg.get("trajectory", {}).get("type", "")

    plotter.attitude_comparison(
        truth.t, truth_euler, filter_euler,
        out_path=fig_dir / f"attitude_comparison.{fmt}",
        disturbance_spans=dist_spans,
        traj_type=traj_type,
    )
    plotter.attitude_error(
        truth.t, filter_errors,
        out_path=fig_dir / f"attitude_error.{fmt}",
        disturbance_spans=dist_spans,
    )
    plotter.geodesic_error(
        truth.t, geo_errors,
        out_path=fig_dir / f"geodesic_error.{fmt}",
        disturbance_spans=dist_spans,
    )
    plotter.rmse_bar(
        rmse_table,
        out_path=fig_dir / f"rmse_bar.{fmt}",
    )
    plotter.convergence(
        truth.t, geo_errors,
        out_path=fig_dir / f"convergence.{fmt}",
    )
    plotter.euler_trajectory_3d(
        truth_euler, filter_euler,
        traj_type=traj_type,
        out_path=fig_dir / f"euler_trajectory_3d.{fmt}",
    )
    plotter.body_axis_sphere(
        truth_euler, filter_euler,
        traj_type=traj_type,
        out_path=fig_dir / f"body_axis_sphere.{fmt}",
    )
    plotter.gt_vs_est_scatter(
        truth_euler, filter_euler,
        out_path=fig_dir / f"scatter_plots.{fmt}",
    )

    # freq_sweep_error — sinusoidal trajectory에서만 의미 있음
    # TODO: freq sweep 결과를 est_results에 포함시킨 후 연결
    # if traj_type == "sinusoidal" and "freq_rmse" in est_results:
    #     plotter.freq_sweep_error(
    #         est_results["freq_rmse"],
    #         out_path=fig_dir / f"freq_sweep_error.{fmt}",
    #     )

# ── 리포트 저장 ───────────────────────────────────────────────────────────────

def _save_report(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    report_dir: Path,
) -> None:
    """FilterReport를 사용하여 MD / JSON / HTML 저장."""
    # FilterReport가 기대하는 형태: {filter_name: {rmse_deg, geodesic_rmse, ...}}
    filter_summary = {
        name: {
            "rmse_deg":         res["rmse_deg"],
            "geodesic_rmse":    res["geodesic_rmse"],
            "geodesic_rmse_deg": res["geodesic_rmse"],   # reporter 호환
        }
        for name, res in est_results.get("filters", {}).items()
    }

    noise_summary = {
        k: {kk: vv for kk, vv in v.items()
            if kk not in ("tau", "adev", "freq", "power", "amp")}
        for k, v in char_results.get("noise", {}).items()
    }
    noise_summary.update(char_results.get("calibration", {}))

    report = FilterReport(filter_summary, noise_summary, cfg)
    report.save(report_dir, format="markdown")
    report.save(report_dir, format="json")
    report.save(report_dir, format="html")


# ── 외란 구간 수집 ────────────────────────────────────────────────────────────

def _collect_disturbance_spans(cfg: dict) -> list[tuple[float, float]] | None:
    spans    = []
    dist_cfg = cfg.get("sensor", {}).get("disturbance", {})
    for key in ("linear_acceleration", "angular_velocity", "magnetic"):
        sec = dist_cfg.get(key, {})
        if sec.get("enabled", False):
            for ev in sec.get("events", []):
                t0 = float(ev["onset_time_s"])
                spans.append((t0, t0 + float(ev["duration_s"])))
    return spans if spans else None