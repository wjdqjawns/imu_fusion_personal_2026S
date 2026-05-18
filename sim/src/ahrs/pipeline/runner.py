"""
File Name: ./src/ahrs/pipeline/runner.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  멀티-스테이지 시뮬레이션 파이프라인

    Stage 1 — Noise characterization (calibration.enabled: true일 때)
        정적 궤적  : gyro / accel / mag Allan variance + PSD
        구형 궤적  : mag hard/soft iron 타원체 피팅
        저장       : data/noise_static.csv, data/noise_sphere.csv
        그림       : fig/allan_*.png, fig/psd_*.png, fig/mag_calibration.png

    Stage 2 — State estimation
        설정 궤적  : GT 생성 → 센서 측정 → 필터 업데이트
        저장       : data/truth.csv, data/{filter}.csv
        그림       : fig/attitude_*.png, fig/geodesic_error.png, etc.

    Report     : report/report.md + report.json (Stage 1 + 2 통합)
"""

from __future__ import annotations

import csv
import logging
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor, GroundTruth
from ahrs.core.trajectory.base import TruthData
from ahrs.core.filters.base import AhrsFilter
from ahrs.core.filters.complementary import ComplementaryFilter
from ahrs.core.filters.mahony import MahonyFilter
from ahrs.core.filters.madgwick import MadgwickFilter
from ahrs.core.filters.ekf import EkfFilter
from ahrs.core.evaluation.metrics import (
    euler_rmse, geodesic_rmse, geodesic_errors, attitude_error_euler
)
from ahrs.core.evaluation.report import FilterReport
from ahrs.core.orientation.transforms import Transforms
from ahrs.utils.configger import load_config

logger = logging.getLogger("ahrs_sim.runner")

# trajectory factory
def build_trajectory(cfg: dict) -> Any:
    from ahrs.core.trajectory.static import StaticTrajectory
    from ahrs.core.trajectory.circular import CircularTrajectory
    from ahrs.core.trajectory.figure8 import Figure8Trajectory
    from ahrs.core.trajectory.spherical import SphericalTrajectory

    traj_type = cfg["trajectory"]["type"]
    traj_cfg  = cfg["trajectory"].get(traj_type, {})

    if traj_type == "static":
        return StaticTrajectory.from_config(traj_cfg)
    elif traj_type == "circular":
        return CircularTrajectory.from_config(traj_cfg)
    elif traj_type == "figure8":
        return Figure8Trajectory.from_config(traj_cfg)
    elif traj_type == "spherical":
        return SphericalTrajectory.from_config(traj_cfg)
    else:
        raise ValueError(f"Unknown trajectory type: {traj_type!r}")

# filter factory 
def build_filters(cfg: dict, env: Environment) -> dict[str, AhrsFilter]:
    run_list = cfg["filters"].get("run", ["complementary", "mahony", "madgwick", "ekf"])
    filt_cfg = cfg["filters"]
    g        = env.gravity_magnitude
    mag_ref  = env.mag_ned

    filters: dict[str, AhrsFilter] = {}

    if "complementary" in run_list:
        filters["complementary"] = ComplementaryFilter(
            **filt_cfg.get("complementary", {})
        )
    if "mahony" in run_list:
        filters["mahony"] = MahonyFilter(**filt_cfg.get("mahony", {}))
    if "madgwick" in run_list:
        filters["madgwick"] = MadgwickFilter(**filt_cfg.get("madgwick", {}))
    if "ekf" in run_list:
        ekf_cfg = dict(filt_cfg.get("ekf", {}))
        ekf_cfg["gravity_m_s2"] = g
        ekf_cfg["mag_ref_uT"]   = mag_ref
        filters["ekf"] = EkfFilter(**ekf_cfg)

    return filters

# ── helpers ───────────────────────────────────────────────────────────────────
def _is_in_event(t: float, event: dict) -> bool:
    onset = event["onset_time_s"]
    return onset <= t < onset + event["duration_s"]

def _collect_raw(traj, env: Environment, sensor: IMUSensor,
                 dt: float, duration: float) -> tuple:
    """trajectory → raw 센서 데이터 수집. (t, gyro, accel, mag) 반환."""
    truth = traj.generate(dt, duration)
    n = len(truth.t)
    gyro  = np.zeros((n, 3))
    accel = np.zeros((n, 3))
    mag   = np.zeros((n, 3))
    for k in range(n):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas = sensor.measure(gt, truth.t[k], dt)
        gyro[k]  = meas.gyro
        accel[k] = meas.accel
        mag[k]   = meas.mag
    return truth.t, gyro, accel, mag


def _make_output_dir(cfg: dict) -> Path:
    out_cfg  = cfg.get("output", {})
    base_dir = Path(out_cfg.get("base_dir", "export"))
    if out_cfg.get("timestamp_dirs", True):
        ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        out_dir = base_dir / ts
    else:
        out_dir = base_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def _build_plotter(cfg: dict):
    from ahrs.utils.plotter import Plotter
    fig_cfg = cfg.get("output", {}).get("fig", {})
    dpi = int(fig_cfg.get("dpi", 150))
    fmt = fig_cfg.get("format", "png")
    return Plotter(dpi=dpi, fmt=fmt)


def _perturb_quaternion(q: np.ndarray, roll_err: float,
                        pitch_err: float, yaw_err: float) -> np.ndarray:
    dq = Transforms.euler_to_quat(roll_err, pitch_err, yaw_err)
    return Transforms.quat_normalize(Transforms.quat_mult(q, dq))


def _q_to_euler_deg(q: np.ndarray) -> np.ndarray:
    return np.rad2deg(Transforms.quat_to_euler(q))

# ── Stage 1: Noise characterization ──────────────────────────────────────────
def _run_noise_stage(cfg: dict, env: Environment, sensor: IMUSensor,
                     dt: float, out_dir: Path, plotter) -> dict:
    """
    정적 + 구형 궤적에서 노이즈 특성 분석.

    Returns:
        {
          "allan_gyro":  {arw_*, bi_*, rrw_*},
          "allan_accel": {arw_*, bi_*, rrw_*},
          "mag_cal":     {hard_iron_uT, soft_iron_inv}
        }
    """
    from ahrs.core.trajectory.static import StaticTrajectory
    from ahrs.core.trajectory.spherical import SphericalTrajectory
    from ahrs.core.evaluation.allan import AllanVariance
    from ahrs.core.calibration.mag_cal import MagCalibrator

    cal_cfg  = cfg.get("calibration", {})
    fig_dir  = out_dir / "fig"
    data_dir = out_dir / "data"
    fig_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    fmt = plotter.fmt
    results: dict = {}

    # ── Allan variance + PSD (정적 궤적) ─────────────────────────────────────
    av_cfg = cal_cfg.get("allan_variance", {})
    if av_cfg.get("enabled", True):
        static_dur = float(av_cfg.get("duration_s", 300.0))
        logger.info("[Stage1] Static data: %.0fs @ %.0fHz = %d samples",
                    static_dur, 1/dt, int(static_dur/dt))

        t_s, gyro_s, accel_s, mag_s = _collect_raw(
            StaticTrajectory(), env, sensor, dt, static_dur
        )
        _save_noise_static_csv(data_dir, t_s, gyro_s, accel_s, mag_s)

        # Allan variance
        av_g = AllanVariance(gyro_s, dt)
        tau_g, adev_g = av_g.compute()
        results["allan_gyro"] = av_g.extract_params()

        av_a = AllanVariance(accel_s, dt)
        tau_a, adev_a = av_a.compute()
        results["allan_accel"] = av_a.extract_params()

        plotter.allan_variance(tau_g, adev_g, axis_labels=["x", "y", "z"],
                               out_path=fig_dir / f"allan_gyro.{fmt}")
        plotter.allan_variance(tau_a, adev_a, axis_labels=["x", "y", "z"],
                               out_path=fig_dir / f"allan_accel.{fmt}")

        # PSD
        fs = 1.0 / dt
        plotter.psd(gyro_s, fs, title="Gyroscope PSD",
                    axis_labels=["x","y","z"], unit_label="(rad/s)²/Hz",
                    out_path=fig_dir / f"psd_gyro.{fmt}")
        plotter.psd(accel_s, fs, title="Accelerometer PSD",
                    axis_labels=["x","y","z"], unit_label="(m/s²)²/Hz",
                    out_path=fig_dir / f"psd_accel.{fmt}")
        plotter.psd(mag_s, fs, title="Magnetometer PSD (static)",
                    axis_labels=["x","y","z"], unit_label="µT²/Hz",
                    out_path=fig_dir / f"psd_mag.{fmt}")

        logger.info("[Stage1] Allan done. Gyro ARW_x=%.4f deg/sqrt(h)",
                    results["allan_gyro"].get("arw_deg_per_sqrth_x", 0))

    # ── Mag hard/soft iron (구형 궤적) ────────────────────────────────────────
    mag_cal_cfg = cal_cfg.get("magnetometer", {})
    if mag_cal_cfg.get("enabled", True):
        sphere_dur = float(mag_cal_cfg.get("duration_s", 120.0))
        logger.info("[Stage1] Spherical data: %.0fs @ %.0fHz = %d samples",
                    sphere_dur, 1/dt, int(sphere_dur/dt))

        t_sp, _, _, mag_sp = _collect_raw(
            SphericalTrajectory(rate_deg_s=30.0), env, sensor, dt, sphere_dur
        )
        _save_noise_sphere_csv(data_dir, t_sp, mag_sp)

        method  = mag_cal_cfg.get("method", "ellipsoid_fit")
        mag_cal = MagCalibrator(method=method)
        mag_cal.fit(mag_sp)
        mag_sp_cal = np.array([mag_cal.apply(m) for m in mag_sp])

        results["mag_cal"] = mag_cal.report()

        plotter.mag_sphere(mag_sp, mag_sp_cal,
                           out_path=fig_dir / f"mag_calibration.{fmt}")

        raw_r = np.linalg.norm(mag_sp, axis=1)
        cal_r = np.linalg.norm(mag_sp_cal, axis=1)
        logger.info("[Stage1] Mag cal done. |m| raw=%.2f±%.3f  cal=%.2f±%.3f uT",
                    raw_r.mean(), raw_r.std(), cal_r.mean(), cal_r.std())

    return results

# ── Stage 2: State estimation ─────────────────────────────────────────────────
def _run_estimation_stage(cfg: dict, env: Environment, sensor: IMUSensor,
                          dt: float, duration: float, out_dir: Path,
                          plotter) -> dict:
    """필터 추정 파이프라인. data/ + fig/ 저장 후 eval_results 반환."""
    fig_dir  = out_dir / "fig"
    data_dir = out_dir / "data"
    fig_dir.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    fmt = plotter.fmt

    # trajectory + filters
    traj             = build_trajectory(cfg)
    truth: TruthData = traj.generate(dt, duration)
    logger.info("[Stage2] Trajectory: %s, steps=%d",
                cfg["trajectory"]["type"], len(truth.t))

    filters = build_filters(cfg, env)
    logger.info("[Stage2] Filters: %s", list(filters.keys()))

    # disturbance events
    dist_cfg   = cfg["imu"].get("disturbance", {})
    lin_events = (dist_cfg.get("linear_acceleration", {}).get("events", [])
                  if dist_cfg.get("linear_acceleration", {}).get("enabled", False) else [])
    mag_events = (dist_cfg.get("magnetic", {}).get("events", [])
                  if dist_cfg.get("magnetic", {}).get("enabled", False) else [])

    # initial conditions
    init_cfg = cfg.get("initial_conditions", {})
    init_att = init_cfg.get("attitude_error_rad", {})
    init_q = _perturb_quaternion(
        truth.q[0],
        roll_err=float(init_att.get("roll", 0.0)),
        pitch_err=float(init_att.get("pitch", 0.0)),
        yaw_err=float(init_att.get("yaw", 0.0)),
    )
    init_bias_err = np.deg2rad(np.array(
        init_cfg.get("gyro_bias_error_deg_s", [0.0, 0.0, 0.0]), dtype=float
    ))
    for filt in filters.values():
        filt._q    = init_q.copy()
        filt._bias = init_bias_err.copy()

    # main loop
    n = len(truth.t)
    filt_results: dict[str, dict] = {
        name: {"q": np.zeros((n, 4)), "bias": np.zeros((n, 3))}
        for name in filters
    }
    raw_gyro  = np.zeros((n, 3))
    raw_accel = np.zeros((n, 3))
    raw_mag   = np.zeros((n, 3))

    for k, t_k in enumerate(truth.t):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas = sensor.measure(gt, t_k, dt)

        accel_meas = meas.accel.copy()
        for ev in lin_events:
            if _is_in_event(t_k, ev):
                direction = np.array(ev["direction_body"], dtype=float)
                direction = direction / (np.linalg.norm(direction) + 1e-12)
                accel_meas += direction * ev["magnitude_g"] * env.gravity_magnitude

        mag_meas = meas.mag.copy()
        for ev in mag_events:
            if _is_in_event(t_k, ev):
                mag_meas += np.array(ev["disturbance_uT"], dtype=float)

        raw_gyro[k]  = meas.gyro
        raw_accel[k] = accel_meas
        raw_mag[k]   = mag_meas

        for name, filt in filters.items():
            q, bias = filt.update(meas.gyro, accel_meas, mag_meas, dt)
            filt_results[name]["q"][k]    = q
            filt_results[name]["bias"][k] = bias

    # evaluation
    eval_results: dict[str, dict] = {}
    for name, res in filt_results.items():
        geo_err = geodesic_errors(res["q"], truth.q)
        e_euler = attitude_error_euler(res["q"], truth.q)
        eval_results[name] = {
            "q":                 res["q"],
            "bias":              res["bias"],
            "geodesic_err_deg":  geo_err,
            "euler_err_deg":     e_euler,
            "rmse_deg":          euler_rmse(
                np.array([Transforms.quat_to_euler(q) for q in res["q"]]),
                np.array([Transforms.quat_to_euler(q) for q in truth.q])
            ).tolist(),
            "geodesic_rmse_deg": geodesic_rmse(res["q"], truth.q),
        }
        logger.info("[Stage2][%s] Geodesic RMSE=%.3f deg",
                    name, eval_results[name]["geodesic_rmse_deg"])

    # save data CSVs
    _save_estimation_csv(data_dir, truth, raw_gyro, raw_accel, raw_mag, eval_results)

    # save comparison figures
    truth_euler   = np.rad2deg(np.array([Transforms.quat_to_euler(q) for q in truth.q]))
    filter_euler  = {n: np.rad2deg(np.array([Transforms.quat_to_euler(q)
                                             for q in r["q"]])) for n, r in eval_results.items()}
    filter_errors = {n: r["euler_err_deg"]   for n, r in eval_results.items()}
    geo_errors    = {n: r["geodesic_err_deg"] for n, r in eval_results.items()}
    rmse_table    = {
        n: {"roll": r["rmse_deg"][0], "pitch": r["rmse_deg"][1],
            "yaw":  r["rmse_deg"][2], "geodesic": r["geodesic_rmse_deg"]}
        for n, r in eval_results.items()
    }

    # disturbance spans for shading
    dist_spans: list[tuple[float, float]] | None = None
    spans = []
    for key in ("linear_acceleration", "magnetic"):
        sec = cfg["imu"].get("disturbance", {}).get(key, {})
        if sec.get("enabled", False):
            for ev in sec.get("events", []):
                t0 = float(ev["onset_time_s"])
                spans.append((t0, t0 + float(ev["duration_s"])))
    if spans:
        dist_spans = spans

    plotter.attitude_comparison(truth.t, truth_euler, filter_euler,
                                out_path=fig_dir / f"attitude_comparison.{fmt}",
                                disturbance_spans=dist_spans)
    plotter.attitude_error(truth.t, filter_errors,
                           out_path=fig_dir / f"attitude_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.geodesic_error(truth.t, geo_errors,
                           out_path=fig_dir / f"geodesic_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.rmse_bar(rmse_table, out_path=fig_dir / f"rmse_bar.{fmt}")
    plotter.trajectory_3d(truth_euler, filter_euler,
                          out_path=fig_dir / f"trajectory_3d.{fmt}")
    plotter.convergence(truth.t, geo_errors,
                        out_path=fig_dir / f"convergence.{fmt}")

    return {"truth": truth, "results": eval_results}


# ── main run function ─────────────────────────────────────────────────────────

def run(config_path: str | Path, dry_run: bool = False,
        log_level: str = "INFO") -> dict:
    """
    전체 파이프라인 실행 (Stage 1 + Stage 2).

    Args:
        config_path: config.yaml 경로
        dry_run:     True이면 설정 검증만
        log_level:   콘솔 로그 레벨

    Returns:
        {
          "truth":      TruthData,
          "results":    eval_results dict,
          "noise":      noise stage results,
          "config":     cfg,
          "out_dir":    Path
        }
    """
    logging.basicConfig(level=getattr(logging, log_level.upper(), logging.INFO),
                        format="[%(levelname)s] %(message)s")

    cfg      = load_config(config_path)
    sim_cfg  = cfg["simulation"]
    dt       = float(sim_cfg["dt"])
    duration = float(sim_cfg["duration"])
    seed     = int(sim_cfg.get("random_seed", 42))

    if dry_run:
        logger.info("Dry run — config OK. dt=%.4fs duration=%.1fs seed=%d",
                    dt, duration, seed)
        return {"dry_run": True}

    # ── setup ─────────────────────────────────────────────────────────────────
    out_dir = _make_output_dir(cfg)

    log_dir = out_dir / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    fh = logging.FileHandler(log_dir / "sim.log", encoding="utf-8")
    fh.setFormatter(logging.Formatter("[%(levelname)s] %(name)s — %(message)s"))
    logging.getLogger().addHandler(fh)

    plotter = _build_plotter(cfg)

    env    = Environment.from_config(cfg["env"])
    sensor = IMUSensor.from_config(cfg, seed=seed)
    logger.info("Env — gravity: %.5f m/s²  mag: %s uT",
                env.gravity_magnitude, np.round(env.mag_ned, 2).tolist())

    # ── Stage 1: Noise characterization ───────────────────────────────────────
    noise_results: dict = {}
    if cfg.get("calibration", {}).get("enabled", True):
        logger.info("=== Stage 1: Noise characterization ===")
        noise_results = _run_noise_stage(cfg, env, sensor, dt, out_dir, plotter)
    else:
        logger.info("Stage 1 skipped (calibration.enabled: false)")

    # ── Stage 2: State estimation ──────────────────────────────────────────────
    logger.info("=== Stage 2: State estimation ===")
    est = _run_estimation_stage(cfg, env, sensor, dt, duration, out_dir, plotter)

    # ── Report ──────────────────────────────────────────────────────────────────
    report = FilterReport(est["results"], noise_results=noise_results, config=cfg)
    report.save(out_dir / "report", fmt="markdown")
    report.save(out_dir / "report", fmt="json")
    logger.info("Report saved → %s", out_dir / "report")
    logger.info("Run complete → %s", out_dir)

    return {
        "truth":   est["truth"],
        "results": est["results"],
        "noise":   noise_results,
        "config":  cfg,
        "out_dir": out_dir,
    }

# CSV save helpers
def _save_noise_static_csv(data_dir: Path, t: np.ndarray,
                            gyro: np.ndarray, accel: np.ndarray,
                            mag: np.ndarray) -> None:
    with (data_dir / "noise_static.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "gx_rad_s", "gy_rad_s", "gz_rad_s",
                    "ax_m_s2", "ay_m_s2", "az_m_s2",
                    "mx_uT", "my_uT", "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *gyro[k], *accel[k], *mag[k]])

def _save_noise_sphere_csv(data_dir: Path, t: np.ndarray,
                            mag: np.ndarray) -> None:
    with (data_dir / "noise_sphere.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "mx_uT", "my_uT", "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *mag[k]])

def _save_estimation_csv(data_dir: Path, truth: TruthData,
                          raw_gyro: np.ndarray, raw_accel: np.ndarray,
                          raw_mag: np.ndarray, results: dict) -> None:
    t = truth.t

    with (data_dir / "truth.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "roll_deg", "pitch_deg", "yaw_deg",
                    "gx_rad_s", "gy_rad_s", "gz_rad_s",
                    "ax_m_s2", "ay_m_s2", "az_m_s2",
                    "mx_uT", "my_uT", "mz_uT"])
        for k in range(len(t)):
            rpy = np.rad2deg(Transforms.quat_to_euler(truth.q[k]))
            w.writerow([t[k], *rpy, *raw_gyro[k], *raw_accel[k], *raw_mag[k]])

    for name, res in results.items():
        with (data_dir / f"{name}.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "roll_deg", "pitch_deg", "yaw_deg",
                        "err_roll_deg", "err_pitch_deg", "err_yaw_deg",
                        "geo_err_deg", "bgx", "bgy", "bgz"])
            for k in range(len(t)):
                rpy = np.rad2deg(Transforms.quat_to_euler(res["q"][k]))
                w.writerow([t[k], *rpy,
                             *res["euler_err_deg"][k],
                             res["geodesic_err_deg"][k],
                             *res["bias"][k]])