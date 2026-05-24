"""
File Name: ./src/ahrs/pipeline/stage_estimation.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 6 — State Estimation

    흐름:
        GT 생성 (trajectory) →
        센서 측정 (보정된 sensor) →
        필터별 추정 (complementary, mahony, madgwick, ekf, ekf_euler, eskf) →
        평가 (Euler RMSE, Geodesic RMSE, 오차 시계열)

    입력:
        cfg     : config dict
        env     : Environment
        sensor  : IMUSensor  (Stage 5에서 apply_calibration 완료된 상태)
        dt      : float
        duration: float
        ctx     : ExportContext
        plotter : Plotter

    출력 dict:
        {
          "truth": TruthData,
          "imu_raw":       {"gyro": ndarray(N,3), "accel": ndarray(N,3), "mag": ndarray(N,3)},
          "imu_calibrated": {"gyro": ..., "accel": ..., "mag": ...},
          "filters": {
              "ekf": {
                  "q":             ndarray(N,4),   # scalar-last [x,y,z,w]
                  "bias":          ndarray(N,3),
                  "euler_deg":     ndarray(N,3),   # [roll, pitch, yaw] deg
                  "euler_err_deg": ndarray(N,3),
                  "geodesic_err":  ndarray(N,),    # deg
                  "rmse_deg":      [roll, pitch, yaw],
                  "geodesic_rmse": float,
              },
              "eskf": {...},
              ...
          }
        }

    side effects:
        - data/estimation/truth.csv
        - data/estimation/imu_raw.csv
        - data/estimation/imu_calibrated.csv
        - data/estimation/{filter_name}.csv
        - fig/estimation/*.png  (runner에서 stage_report가 담당)
"""

from __future__ import annotations

import csv
import logging
from pathlib import Path

import numpy as np

from ahrs.pipeline.builder import build_trajectory, build_estimators
from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor, GroundTruth
from ahrs.core.trajectory.base import TruthData
from ahrs.utils.transforms import Transforms
from ahrs.core.evaluation.metrics import (
    euler_rmse, geodesic_rmse, geodesic_errors, attitude_error_euler
)

logger = logging.getLogger("ahrs_sim.estimation")

def run_estimation(
    cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
    ctx,      # ExportContext
    plotter,  # Plotter
) -> dict:
    """
    Stage 6 전체 실행.

    sensor는 Stage 5에서 apply_calibration()이 완료된 상태여야 함.
    raw IMU 출력과 캘리브레이션 적용 후 출력을 모두 저장해
    보정 효과를 사후 분석할 수 있도록 함.
    """
    est_dir = ctx.data_dir / "estimation"
    fig_dir = ctx.fig_dir  / "estimation"
    for d in (est_dir, fig_dir):
        d.mkdir(parents=True, exist_ok=True)

    # ── GT 생성 ───────────────────────────────────────────────────────────────
    traj  = build_trajectory(cfg)
    truth = traj.generate(dt, duration)
    logger.info("[Stage6] Trajectory: %s | steps=%d",
                cfg["trajectory"]["type"], len(truth.t))

    # ── 필터 빌드 ─────────────────────────────────────────────────────────────
    filters = build_estimators(cfg, env)
    logger.info("[Stage6] Filters: %s", list(filters.keys()))

    # ── 초기 조건 주입 ────────────────────────────────────────────────────────
    _apply_initial_conditions(cfg, truth, filters)

    # ── 메인 추정 루프 ────────────────────────────────────────────────────────
    n = len(truth.t)
    filt_q    = {name: np.zeros((n, 4)) for name in filters}
    filt_bias = {name: np.zeros((n, 3)) for name in filters}

    # 보정 전/후 IMU 저장용
    raw_gyro  = np.zeros((n, 3))
    raw_accel = np.zeros((n, 3))
    raw_mag   = np.zeros((n, 3))
    cal_gyro  = np.zeros((n, 3))
    cal_accel = np.zeros((n, 3))
    cal_mag   = np.zeros((n, 3))

    dist_cfg = cfg.get("sensor", {}).get("disturbance", {})

    for k, t_k in enumerate(truth.t):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )

        # 센서 측정 (보정 전 raw)
        meas = sensor.measure(gt, t_k, dt)
        raw_gyro[k]  = meas.gyro
        raw_accel[k] = meas.accel
        raw_mag[k]   = meas.mag

        # 보정 적용 (sensor.apply_calibration이 내부 파이프라인에 적용)
        # 여기서는 disturbance 추가 후 calibrated 값으로 간주
        accel_in = meas.accel.copy()
        mag_in   = meas.mag.copy()
        gyro_in  = meas.gyro.copy()

        # 외란 주입 (측정값에 추가, GT에는 반영 안 함)
        gyro_in, accel_in, mag_in = _apply_disturbance(
            t_k, gyro_in, accel_in, mag_in, dist_cfg
        )

        cal_gyro[k]  = gyro_in
        cal_accel[k] = accel_in
        cal_mag[k]   = mag_in

        # 필터 업데이트
        for name, filt in filters.items():
            q, bias = filt.update(gyro_in, accel_in, mag_in, dt)
            filt_q[name][k]    = q
            filt_bias[name][k] = bias

    # ── 평가 ──────────────────────────────────────────────────────────────────
    filter_results = _evaluate(truth, filt_q, filt_bias)

    # ── 저장 ──────────────────────────────────────────────────────────────────
    _save_truth_csv(est_dir, truth)
    _save_imu_csv(est_dir / "imu_raw.csv",
                  truth.t, raw_gyro, raw_accel, raw_mag, label="raw")
    _save_imu_csv(est_dir / "imu_calibrated.csv",
                  truth.t, cal_gyro, cal_accel, cal_mag, label="calibrated")
    _save_filter_csvs(est_dir, truth.t, filter_results)

    logger.info("[Stage6] Estimation complete")
    for name, res in filter_results.items():
        logger.info("[Stage6] [%s] Geodesic RMSE=%.3f deg  Euler RMSE=[%.3f, %.3f, %.3f] deg",
                    name, res["geodesic_rmse"], *res["rmse_deg"])

    return {
        "truth":          truth,
        "imu_raw":        {"gyro": raw_gyro,  "accel": raw_accel,  "mag": raw_mag},
        "imu_calibrated": {"gyro": cal_gyro,  "accel": cal_accel,  "mag": cal_mag},
        "filters":        filter_results,
    }

def _evaluate(
    truth: TruthData,
    filt_q: dict[str, np.ndarray],
    filt_bias: dict[str, np.ndarray],
) -> dict:
    """필터별 오차 계산."""
    results = {}
    for name, q_est in filt_q.items():
        geo_err   = geodesic_errors(q_est, truth.q)       # (N,) deg
        euler_err = attitude_error_euler(q_est, truth.q)  # (N,3) deg
        euler_est = np.rad2deg(np.array(
            [Transforms.quat_to_euler(q) for q in q_est]
        ))                                                 # (N,3) deg
        results[name] = {
            "q":             q_est,
            "bias":          filt_bias[name],
            "euler_deg":     euler_est,
            "euler_err_deg": euler_err,
            "geodesic_err":  geo_err,
            "rmse_deg":      euler_rmse(
                euler_est,
                np.rad2deg(np.array(
                    [Transforms.quat_to_euler(q) for q in truth.q]
                ))
            ).tolist(),
            "geodesic_rmse": float(geodesic_rmse(q_est, truth.q)),
        }
    return results

def _apply_initial_conditions(cfg: dict, truth: TruthData, filters: dict) -> None:
    init_cfg = cfg.get("estimator", {}).get("initial_conditions", {})
    att_err  = init_cfg.get("attitude_error_deg", {})
    bias_err = np.deg2rad(
        np.array(init_cfg.get("gyro_bias_error_deg_s", [0.0, 0.0, 0.0]),
                 dtype=float)
    )

    roll_err  = np.deg2rad(float(att_err.get("roll",  0.0)))
    pitch_err = np.deg2rad(float(att_err.get("pitch", 0.0)))
    yaw_err   = np.deg2rad(float(att_err.get("yaw",   0.0)))

    dq    = Transforms.euler_to_quat(roll_err, pitch_err, yaw_err)
    q_init = Transforms.quat_normalize(
        Transforms.quat_mult(truth.q[0], dq)
    )

    for filt in filters.values():
        filt._q    = q_init.copy()
        filt._bias = bias_err.copy()

def _apply_disturbance(
    t: float,
    gyro: np.ndarray,
    accel: np.ndarray,
    mag: np.ndarray,
    dist_cfg: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """외란 이벤트를 측정값에 추가 (GT에는 반영 안 함)."""

    # 선형 가속도 외란 → accel correction 비활성화 유발
    la_cfg = dist_cfg.get("linear_acceleration", {})
    if la_cfg.get("enabled", False):
        for ev in la_cfg.get("events", []):
            if _in_event(t, ev):
                direction = np.array(ev["direction_body"], dtype=float)
                direction /= np.linalg.norm(direction) + 1e-12
                accel = accel + direction * ev["magnitude_g"] * 9.80665

    # 각속도 외란 → gyro predict 오염
    av_cfg = dist_cfg.get("angular_velocity", {})
    if av_cfg.get("enabled", False):
        for ev in av_cfg.get("events", []):
            if _in_event(t, ev):
                axis = np.array(ev["axis_body"], dtype=float)
                axis /= np.linalg.norm(axis) + 1e-12
                gyro = gyro + axis * np.deg2rad(ev["magnitude_deg_s"])

    # 자기 외란 → mag correction 교란 (yaw 추정 오류)
    mag_cfg = dist_cfg.get("magnetic", {})
    if mag_cfg.get("enabled", False):
        for ev in mag_cfg.get("events", []):
            if _in_event(t, ev):
                mag = mag + np.array(ev["disturbance_uT"], dtype=float)

    return gyro, accel, mag

def _in_event(t: float, event: dict) -> bool:
    onset = float(event["onset_time_s"])
    return onset <= t < onset + float(event["duration_s"])

def _save_truth_csv(est_dir: Path, truth: TruthData) -> None:
    path = est_dir / "truth.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s",
                    "roll_deg", "pitch_deg", "yaw_deg",
                    "qx", "qy", "qz", "qw",
                    "wx_rad_s", "wy_rad_s", "wz_rad_s",
                    "px_m", "py_m", "pz_m",
                    "vx_m_s", "vy_m_s", "vz_m_s"])
        for k in range(len(truth.t)):
            rpy = np.rad2deg(Transforms.quat_to_euler(truth.q[k]))
            q   = truth.q[k]   # [x,y,z,w] scalar-last
            w.writerow([truth.t[k],
                        *rpy, *q,
                        *truth.omega[k],
                        *truth.pos[k],
                        *truth.vel[k]])
    logger.debug("[Stage6] Saved truth → %s", path)

def _save_imu_csv(
    path: Path,
    t: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    mag: np.ndarray,
    label: str = "raw",
) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s",
                    "gx_rad_s", "gy_rad_s", "gz_rad_s",
                    "ax_m_s2",  "ay_m_s2",  "az_m_s2",
                    "mx_uT",    "my_uT",    "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *gyro[k], *accel[k], *mag[k]])
    logger.debug("[Stage6] Saved IMU %s → %s", label, path)

def _save_filter_csvs(
    est_dir: Path,
    t: np.ndarray,
    results: dict,
) -> None:
    for name, res in results.items():
        path = est_dir / f"{name}.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s",
                        "roll_deg", "pitch_deg", "yaw_deg",
                        "qx", "qy", "qz", "qw",
                        "err_roll_deg", "err_pitch_deg", "err_yaw_deg",
                        "geodesic_err_deg",
                        "bias_x_rad_s", "bias_y_rad_s", "bias_z_rad_s"])
            for k in range(len(t)):
                w.writerow([t[k],
                            *res["euler_deg"][k],
                            *res["q"][k],
                            *res["euler_err_deg"][k],
                            res["geodesic_err"][k],
                            *res["bias"][k]])
        logger.debug("[Stage6] Saved filter %s → %s", name, path)

def _run_estimation_stage(
    cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
    ctx: ExportContext,
    plotter: Plotter,
) -> dict:
    """Filter estimation pipeline. Saves data CSVs + figures, returns eval dict."""
    fmt = plotter.fmt

    # trajectory + estimators
    traj_type     = cfg["trajectory"]["type"]
    traj_type_str = traj_type.replace("_", " ").title()
    traj          = _build_trajectory(cfg)
    truth: TruthData = traj.generate(dt, duration)
    logger.info("Trajectory: %s  steps=%d", traj_type, len(truth.t))

    filters = _build_estimator(cfg, env)
    logger.info("Estimators: %s", list(filters.keys()))

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
        roll_err=float(init_att.get("roll",  0.0)),
        pitch_err=float(init_att.get("pitch", 0.0)),
        yaw_err=float(init_att.get("yaw",   0.0)),
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
                d = np.array(ev["direction_body"], dtype=float)
                d = d / (np.linalg.norm(d) + 1e-12)
                accel_meas += d * ev["magnitude_g"] * env.gravity_magnitude

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
            "rmse_deg": euler_rmse(
                np.array([Transforms.quat_to_euler(q) for q in res["q"]]),
                np.array([Transforms.quat_to_euler(q) for q in truth.q]),
            ).tolist(),
            "geodesic_rmse_deg": geodesic_rmse(res["q"], truth.q),
        }
        logger.info("[%s] Geodesic RMSE=%.3f°", name, eval_results[name]["geodesic_rmse_deg"])

    # CSV export
    save_estimation_csv(ctx.data_dir, truth, raw_gyro, raw_accel, raw_mag,
                        eval_results, Transforms.quat_to_euler)

    # figures
    truth_euler  = np.rad2deg(np.array([Transforms.quat_to_euler(q) for q in truth.q]))
    filter_euler = {n: np.rad2deg(np.array([Transforms.quat_to_euler(q)
                                            for q in r["q"]])) for n, r in eval_results.items()}
    filter_errors = {n: r["euler_err_deg"]   for n, r in eval_results.items()}
    geo_errors    = {n: r["geodesic_err_deg"] for n, r in eval_results.items()}
    rmse_table    = {
        n: {"roll": r["rmse_deg"][0], "pitch": r["rmse_deg"][1],
            "yaw":  r["rmse_deg"][2], "geodesic": r["geodesic_rmse_deg"]}
        for n, r in eval_results.items()
    }

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
                                out_path=ctx.fig_dir / f"attitude_comparison.{fmt}",
                                disturbance_spans=dist_spans,
                                traj_type=traj_type_str)
    plotter.attitude_error(truth.t, filter_errors,
                           out_path=ctx.fig_dir / f"attitude_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.geodesic_error(truth.t, geo_errors,
                           out_path=ctx.fig_dir / f"geodesic_error.{fmt}",
                           disturbance_spans=dist_spans)
    plotter.rmse_bar(rmse_table,
                     out_path=ctx.fig_dir / f"rmse_bar.{fmt}")
    plotter.euler_trajectory_3d(truth_euler, filter_euler,
                                traj_type=traj_type_str,
                                out_path=ctx.fig_dir / f"euler_trajectory_3d.{fmt}")
    plotter.body_axis_sphere(truth_euler, filter_euler,
                             traj_type=traj_type_str,
                             out_path=ctx.fig_dir / f"body_axis_sphere.{fmt}")
    plotter.convergence(truth.t, geo_errors,
                        out_path=ctx.fig_dir / f"convergence.{fmt}")
    plotter.gt_vs_est_scatter(truth_euler, filter_euler,
                               out_path=ctx.fig_dir / f"gt_vs_est_scatter.{fmt}")

    if traj_type == "sinusoidal":
        logger.info("Running frequency sweep evaluation ...")
        freq_rmse = _run_freq_sweep(cfg, env, sensor, dt)
        if freq_rmse:
            plotter.freq_sweep_error(freq_rmse,
                                     out_path=ctx.fig_dir / f"freq_sweep_error.{fmt}")
            logger.info("Freq sweep done — %d frequencies", len(freq_rmse))

    return {"truth": truth, "results": eval_results}

# ── Frequency sweep ───────────────────────────────────────────────────────────
def _run_freq_sweep(
    cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
) -> dict[float, dict[str, float]]:
    """
    Run each sinusoidal frequency independently with fresh filters.
    Returns {freq_hz: {filter_name: geodesic_rmse_deg}}.
    """
    from ahrs.core.trajectory.sinusoidal import SinusoidalTrajectory

    sin_cfg     = cfg["trajectory"].get("sinusoidal", {})
    frequencies = sin_cfg.get("frequencies_hz", [])
    if not frequencies:
        return {}

    amp_deg          = float(sin_cfg.get("amplitude_deg", 30.0))
    axes             = sin_cfg.get("axes", ["roll", "pitch", "yaw"])
    phase_offset_deg = sin_cfg.get("phase_offset_deg", [0, 45, 90])
    freq_rmse: dict[float, dict[str, float]] = {}

    for freq in frequencies:
        seg_dur    = max(5.0 / freq, 2.0)
        single_cfg = {
            "frequencies_hz":      [freq],
            "amplitude_deg":       amp_deg,
            "axes":                axes,
            "phase_offset_deg":    phase_offset_deg,
            "duration_per_freq_s": seg_dur,
        }
        traj  = SinusoidalTrajectory.from_config(single_cfg)
        truth = traj.generate(dt, seg_dur)
        n     = len(truth.t)

        filters = _build_estimator(cfg, env)
        filt_q  = {name: np.zeros((n, 4)) for name in filters}

        for k in range(n):
            gt = GroundTruth(
                q=truth.q[k], omega=truth.omega[k],
                pos=truth.pos[k], vel=truth.vel[k],
                accel_world=truth.accel_world[k],
                mag_world=env.mag_ned,
            )
            meas = sensor.measure(gt, truth.t[k], dt)
            for name, filt in filters.items():
                q, _ = filt.update(meas.gyro, meas.accel, meas.mag, dt)
                filt_q[name][k] = q

        skip = min(int(1.0 / dt), n // 2)
        rmse_dict = {
            name: geodesic_rmse(q_est[skip:], truth.q[skip:])
            for name, q_est in filt_q.items()
        }
        freq_rmse[freq] = rmse_dict
        logger.info("%.1f Hz — %s", freq,
                    "  ".join(f"{nm}={v:.2f}°" for nm, v in rmse_dict.items()))

    return freq_rmse

# --- misc helpers ---
def _is_in_event(t: float, event: dict) -> bool:
    onset = event["onset_time_s"]
    return onset <= t < onset + event["duration_s"]

def _collect_raw(
    traj,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Generate trajectory and collect raw sensor measurements."""
    truth = traj.generate(dt, duration)
    n     = len(truth.t)
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

def _perturb_quaternion(
    q: np.ndarray,
    roll_err: float,
    pitch_err: float,
    yaw_err: float,
) -> np.ndarray:
    dq = Transforms.euler_to_quat(roll_err, pitch_err, yaw_err)
    return Transforms.quat_normalize(Transforms.quat_mult(q, dq))


