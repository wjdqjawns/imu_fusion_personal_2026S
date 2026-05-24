"""
File Name: ./src/ahrs/pipeline/stage_report.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 7 — Plot + Report

    Stage 5, 6의 결과를 받아 모든 플롯을 저장하고
    Markdown / JSON 리포트를 생성.

    입력:
        cfg         : config dict
        char_results: Stage 5 출력 dict
        est_results : Stage 6 출력 dict
        ctx         : ExportContext
        plotter     : Plotter

    출력: 없음 (side effect only)
        fig/characterization/*.png
        fig/estimation/*.png
        report/report.md
        report/report.json
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from datetime import datetime

import numpy as np

from ahrs.utils.transforms import Transforms
from ahrs.utils.reporter import FilterReport

logger = logging.getLogger("ahrs_sim.report")

def run_report(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    ctx,      # ExportContext
    plotter,  # Plotter
) -> None:
    """Stage 7 전체 실행."""

    fig_char = ctx.fig_dir / "characterization"
    fig_est  = ctx.fig_dir / "estimation"
    for d in (fig_char, fig_est, ctx.report_dir):
        d.mkdir(parents=True, exist_ok=True)

    fmt = plotter.fmt

    # ── Characterization 플롯 ─────────────────────────────────────────────────
    _plot_characterization(char_results, fig_char, plotter, fmt)

    # ── Estimation 플롯 ───────────────────────────────────────────────────────
    _plot_estimation(cfg, est_results, fig_est, plotter, fmt)

    # ── 리포트 저장 ───────────────────────────────────────────────────────────
    # _save_report(cfg, char_results, est_results, ctx.report_dir)
    export_report(ctx.report_dir, est_results, char_results, cfg)

    logger.info("[Stage7] Report saved | %s", ctx.report_dir)

# ── Characterization 플롯 ─────────────────────────────────────────────────────
def _plot_characterization(
    char_results: dict,
    fig_dir: Path,
    plotter,
    fmt: str,
) -> None:
    noise = char_results.get("noise", {})
    cal   = char_results.get("calibration", {})

    # Allan Variance
    if "allan_gyro" in noise:
        r = noise["allan_gyro"]
        plotter.allan_variance(
            r["tau"], r["adev"], axis_labels=["x", "y", "z"],
            out_path=fig_dir / f"allan_gyro.{fmt}"
        )
    if "allan_accel" in noise:
        r = noise["allan_accel"]
        plotter.allan_variance(
            r["tau"], r["adev"], axis_labels=["x", "y", "z"],
            out_path=fig_dir / f"allan_accel.{fmt}"
        )

    # PSD
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

    # FFT
    for sensor_name in ("gyro", "accel", "mag"):
        key = f"fft_{sensor_name}"
        if key in noise:
            r = noise[key]
            plotter.fft_spectrum(
                r["amp"], freq=r["freq"],
                title=f"{sensor_name.capitalize()} FFT",
                out_path=fig_dir / f"fft_{sensor_name}.{fmt}"
            )

    # Mag 캘리브레이션 (두 장)
    if "mag" in cal:
        # 플롯 1: 지금처럼 점군 + 타원 (궤적/균일성 확인용)
        # plotter.mag_trajectory(raw, calibrated,
        #                        out_path=fig_dir / f"mag_trajectory.{fmt}")

        # 플롯 2: Residual 분석 (보정 품질 정량화)
        # plotter.mag_residual(res_raw, res_cal, mag_cal,
        #                      sphericity_before=..., sphericity_after=...,
        #                      out_path=fig_dir / f"mag_residual.{fmt}")
        pass

    # 온도 보상 플롯
    for sensor_name in ("gyro", "accel", "mag"):
        if sensor_name in cal and cal[sensor_name].get("temp_coeff"):
            pass
            # plotter.temp_compensation(...,
            #     out_path=fig_dir / f"temp_compensation_{sensor_name}.{fmt}")

def _plot_estimation(
    cfg: dict,
    est_results: dict,
    fig_dir: Path,
    plotter,
    fmt: str,
) -> None:
    truth   = est_results["truth"]
    filters = est_results["filters"]

    truth_euler  = np.rad2deg(np.array(
        [Transforms.quat_to_euler(q) for q in truth.q]
    ))
    filter_euler = {
        name: res["euler_deg"] for name, res in filters.items()
    }
    filter_errors = {
        name: res["euler_err_deg"] for name, res in filters.items()
    }
    geo_errors = {
        name: res["geodesic_err"] for name, res in filters.items()
    }
    rmse_table = {
        name: {
            "roll":      res["rmse_deg"][0],
            "pitch":     res["rmse_deg"][1],
            "yaw":       res["rmse_deg"][2],
            "geodesic":  res["geodesic_rmse"],
        }
        for name, res in filters.items()
    }

    # 외란 구간 (음영 표시용)
    dist_spans = _collect_disturbance_spans(cfg)

    plotter.attitude_comparison(
        truth.t, truth_euler, filter_euler,
        out_path=fig_dir / f"attitude_comparison.{fmt}",
        disturbance_spans=dist_spans,
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
    plotter.trajectory_3d(
        truth_euler, filter_euler,
        out_path=fig_dir / f"trajectory_3d.{fmt}",
    )

    # bias 추정 시계열
    bias_data = {name: res["bias"] for name, res in filters.items()
                 if res["bias"].any()}
    if bias_data:
        plotter.bias_estimation(
            truth.t, bias_data,
            out_path=fig_dir / f"bias_estimation.{fmt}",
        )

    # GT vs 추정 scatter (y=x 직선)
    plotter.gt_vs_est_scatter(
        truth_euler, filter_euler,
        out_path=fig_dir / f"scatter_plots.{fmt}",
    )

def _save_report(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    report_dir: Path,
) -> None:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    _save_report_md(cfg, char_results, est_results, report_dir / f"report_{ts}.md")
    _save_report_json(cfg, char_results, est_results, report_dir / f"report_{ts}.json")

def _save_report_md(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    path: Path,
) -> None:
    lines = []
    ts    = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    sim   = cfg.get("simulation", {})

    lines += [
        "# AHRS Simulation Report",
        f"\nGenerated: {ts}",
        "",
        "## 실행 환경",
        f"- dt: {sim.get('dt')} s  ({1/sim.get('dt',1):.0f} Hz)",
        f"- duration: {sim.get('duration')} s",
        f"- random_seed: {sim.get('random_seed')}",
        f"- trajectory: {cfg.get('trajectory', {}).get('type')}",
        "",
    ]

    # Stage 5 — Sensor Characterization
    lines += ["## Stage 5 — Sensor Characterization", ""]
    noise = char_results.get("noise", {})

    # Allan 결과 테이블
    if "allan_gyro" in noise:
        g = noise["allan_gyro"]
        lines += [
            "### Gyroscope Allan Deviation",
            "",
            "| Axis | ARW [deg/√h] | BI [deg/h] | RRW [deg/h/√h] |",
            "|------|-------------|------------|----------------|",
        ]
        for ax in ("x", "y", "z"):
            lines.append(
                f"| {ax} "
                f"| {g.get(f'arw_{ax}', 0):.4f} "
                f"| {g.get(f'bi_{ax}',  0):.4f} "
                f"| {g.get(f'rrw_{ax}', 0):.4f} |"
            )
        lines.append("")

    if "allan_accel" in noise:
        a = noise["allan_accel"]
        lines += [
            "### Accelerometer Allan Deviation",
            "",
            "| Axis | ARW [m/s/√h] | BI | RRW |",
            "|------|-------------|-----|-----|",
        ]
        for ax in ("x", "y", "z"):
            lines.append(
                f"| {ax} "
                f"| {a.get(f'arw_{ax}', 0):.4f} "
                f"| {a.get(f'bi_{ax}',  0):.4f} "
                f"| {a.get(f'rrw_{ax}', 0):.4f} |"
            )
        lines.append("")

    # Mag 캘리브레이션 결과
    cal = char_results.get("calibration", {})
    if "mag" in cal:
        m = cal["mag"]
        lines += [
            "### Magnetometer Calibration",
            "",
            f"- Hard iron offset [µT]: {np.round(m.get('hard_iron_uT', []), 3).tolist()}",
            f"- Sphericity before: {m.get('sphericity_before', 0):.4f}",
            f"- Sphericity after:  {m.get('sphericity_after',  0):.4f}",
            f"- Residual std before: {m.get('residual_std_before', 0):.4f} µT",
            f"- Residual std after:  {m.get('residual_std_after',  0):.4f} µT",
            "",
        ]

    # Stage 6 — Estimation
    lines += ["## Stage 6 — State Estimation", ""]
    filters = est_results.get("filters", {})

    if filters:
        lines += [
            "### RMSE Summary",
            "",
            "| Filter | Roll [deg] | Pitch [deg] | Yaw [deg] | Geodesic [deg] |",
            "|--------|-----------|------------|----------|---------------|",
        ]
        for name, res in filters.items():
            r = res["rmse_deg"]
            lines.append(
                f"| {name} "
                f"| {r[0]:.3f} "
                f"| {r[1]:.3f} "
                f"| {r[2]:.3f} "
                f"| {res['geodesic_rmse']:.3f} |"
            )
        lines.append("")

    path.write_text("\n".join(lines), encoding="utf-8")
    logger.debug("[Stage7] Markdown report → %s", path)

def _save_report_json(
    cfg: dict,
    char_results: dict,
    est_results: dict,
    path: Path,
) -> None:
    def _serialize(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        if isinstance(obj, dict):
            return {k: _serialize(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_serialize(i) for i in obj]
        return obj

    # TruthData는 JSON 직렬화에서 제외 (CSV에 저장됨)
    report = {
        "generated_at": datetime.now().isoformat(),
        "config": {
            "simulation": cfg.get("simulation", {}),
            "trajectory": cfg.get("trajectory", {}).get("type"),
            "estimator":  {"run": cfg.get("estimator", {}).get("run", [])},
        },
        "noise": {
            k: {kk: vv for kk, vv in v.items()
                if kk not in ("tau", "adev", "freq", "power", "amp")}
            for k, v in char_results.get("noise", {}).items()
        },
        "calibration": char_results.get("calibration", {}),
        "estimation": {
            name: {
                "rmse_deg":      res["rmse_deg"],
                "geodesic_rmse": res["geodesic_rmse"],
            }
            for name, res in est_results.get("filters", {}).items()
        },
    }

    path.write_text(
        json.dumps(_serialize(report), indent=2, ensure_ascii=False),
        encoding="utf-8"
    )
    logger.debug("[Stage7] JSON report → %s", path)

def _collect_disturbance_spans(cfg: dict) -> list[tuple[float, float]] | None:
    spans = []
    dist_cfg = cfg.get("sensor", {}).get("disturbance", {})
    for key in ("linear_acceleration", "angular_velocity", "magnetic"):
        sec = dist_cfg.get(key, {})
        if sec.get("enabled", False):
            for ev in sec.get("events", []):
                t0 = float(ev["onset_time_s"])
                spans.append((t0, t0 + float(ev["duration_s"])))
    return spans if spans else None

def export_report(
    report_dir: Path,
    estimation_results: dict,
    noise_results: dict | None = None,
    config: dict | None = None,
) -> None:
    report = FilterReport(estimation_results, noise_results, config)
    report.save(report_dir / "report", format="markdown")
    report.save(report_dir / "report", format="json")
    report.save(report_dir / "report", format="html")