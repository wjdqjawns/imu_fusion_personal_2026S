"""
File Name: ./src/ahrs/pipeline/stage/stage_estimation.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 6 — State Estimation

    흐름:
        GT 생성 (trajectory) →
        센서 측정 (보정된 sensor) →
        필터별 추정 →
        평가 (Euler RMSE, Geodesic RMSE, 오차 시계열)

    입력:
        cfg     : config dict
        env     : Environment
        sensor  : IMUSensor  (Stage 5에서 apply_calibration 완료)
        dt      : float
        duration: float
        ctx     : ExportContext
        plotter : Plotter

    출력 dict:
        {
          "truth": TruthData,
          "imu_raw":        {"gyro": ndarray(N,3), "accel": ndarray(N,3), "mag": ndarray(N,3)},
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
              ...  # 다른 필터 동일 구조
          }
        }

    side effects:
        data/estimation/truth.csv
        data/estimation/imu_raw.csv
        data/estimation/imu_calibrated.csv
        data/estimation/{filter_name}.csv
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from ahrs.pipeline.builder import build_trajectory, build_estimator
from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor, GroundTruth
from ahrs.core.trajectory.base import TruthData
from ahrs.utils.exporter import ExportContext #, save_truth_csv, save_imu_csv, save_filter_csv
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
    ctx: ExportContext,
    plotter,
) -> dict:
    """
    Stage 6 전체 실행.

    sensor는 Stage 5에서 apply_calibration()이 완료된 상태여야 함.
    raw IMU 출력과 캘리브레이션 적용 후 출력을 모두 저장해
    보정 효과를 사후 분석할 수 있도록 함.
    """
    est_dir = ctx.data_dir / "estimation"
    est_dir.mkdir(parents=True, exist_ok=True)

    # ── GT 생성 ───────────────────────────────────────────────────────────────
    traj  = build_trajectory(cfg)
    truth = traj.generate(dt, duration)
    logger.info("[Stage6] Trajectory: %s | steps=%d",
                cfg["trajectory"]["type"], len(truth.t))

    # ── 필터 빌드 ─────────────────────────────────────────────────────────────
    filters = build_estimator(cfg)
    logger.info("[Stage6] Filters: %s", list(filters.keys()))

    # ── 초기 조건 주입 ────────────────────────────────────────────────────────
    _apply_initial_conditions(cfg, truth, filters)

    # ── 메인 추정 루프 ────────────────────────────────────────────────────────
    n = len(truth.t)
    filt_q    = {name: np.zeros((n, 4)) for name in filters}
    filt_bias = {name: np.zeros((n, 3)) for name in filters}

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
        meas = sensor.measure(gt, t_k, dt)

        raw_gyro[k]  = meas.gyro
        raw_accel[k] = meas.accel
        raw_mag[k]   = meas.mag

        gyro_in, accel_in, mag_in = _apply_disturbance(
            t_k, meas.gyro.copy(), meas.accel.copy(), meas.mag.copy(), dist_cfg
        )

        cal_gyro[k]  = gyro_in
        cal_accel[k] = accel_in
        cal_mag[k]   = mag_in

        for name, filt in filters.items():
            q, bias          = filt.update(gyro_in, accel_in, mag_in, dt)
            filt_q[name][k]    = q
            filt_bias[name][k] = bias

    # ── 평가 ──────────────────────────────────────────────────────────────────
    filter_results = _evaluate(truth, filt_q, filt_bias)

    # ── 저장 ──────────────────────────────────────────────────────────────────
    save_truth_csv(est_dir, truth)
    save_imu_csv(est_dir / "imu_raw.csv", truth.t, raw_gyro, raw_accel, raw_mag)
    save_imu_csv(est_dir / "imu_calibrated.csv", truth.t, cal_gyro, cal_accel, cal_mag)
    for name, res in filter_results.items():
        save_filter_csv(est_dir / f"{name}.csv", truth.t, res)

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


# ── 평가 ──────────────────────────────────────────────────────────────────────

def _evaluate(
    truth: TruthData,
    filt_q: dict[str, np.ndarray],
    filt_bias: dict[str, np.ndarray],
) -> dict:
    results = {}
    truth_euler = np.rad2deg(np.array(
        [Transforms.quat_to_euler(q) for q in truth.q]
    ))

    for name, q_est in filt_q.items():
        geo_err   = geodesic_errors(q_est, truth.q)
        euler_err = attitude_error_euler(q_est, truth.q)
        euler_est = np.rad2deg(np.array(
            [Transforms.quat_to_euler(q) for q in q_est]
        ))
        results[name] = {
            "q":             q_est,
            "bias":          filt_bias[name],
            "euler_deg":     euler_est,
            "euler_err_deg": euler_err,
            "geodesic_err":  geo_err,
            "rmse_deg":      euler_rmse(euler_est, truth_euler).tolist(),
            "geodesic_rmse": float(geodesic_rmse(q_est, truth.q)),
        }
    return results


# ── 초기 조건 ─────────────────────────────────────────────────────────────────

def _apply_initial_conditions(cfg: dict, truth: TruthData, filters: dict) -> None:
    init_cfg = cfg.get("estimator", {}).get("initial_conditions", {})
    att_err  = init_cfg.get("attitude_error_deg", {})
    bias_err = np.deg2rad(
        np.array(init_cfg.get("gyro_bias_error_deg_s", [0.0, 0.0, 0.0]), dtype=float)
    )

    roll_err  = np.deg2rad(float(att_err.get("roll",  0.0)))
    pitch_err = np.deg2rad(float(att_err.get("pitch", 0.0)))
    yaw_err   = np.deg2rad(float(att_err.get("yaw",   0.0)))

    dq     = Transforms.euler_to_quat(roll_err, pitch_err, yaw_err)
    q_init = Transforms.quat_normalize(Transforms.quat_mult(truth.q[0], dq))

    for filt in filters.values():
        if hasattr(filt, "_q"):
            filt._q = q_init.copy()
        if hasattr(filt, "_bias"):
            filt._bias = bias_err.copy()


# ── 외란 주입 ─────────────────────────────────────────────────────────────────

def _apply_disturbance(
    t: float,
    gyro: np.ndarray,
    accel: np.ndarray,
    mag: np.ndarray,
    dist_cfg: dict,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """외란 이벤트를 측정값에 추가 (GT에는 반영 안 함)."""
    la_cfg = dist_cfg.get("linear_acceleration", {})
    if la_cfg.get("enabled", False):
        for ev in la_cfg.get("events", []):
            if _in_event(t, ev):
                direction = np.array(ev["direction_body"], dtype=float)
                direction /= np.linalg.norm(direction) + 1e-12
                accel = accel + direction * ev["magnitude_g"] * 9.80665

    av_cfg = dist_cfg.get("angular_velocity", {})
    if av_cfg.get("enabled", False):
        for ev in av_cfg.get("events", []):
            if _in_event(t, ev):
                axis = np.array(ev["axis_body"], dtype=float)
                axis /= np.linalg.norm(axis) + 1e-12
                gyro = gyro + axis * np.deg2rad(ev["magnitude_deg_s"])

    mag_cfg = dist_cfg.get("magnetic", {})
    if mag_cfg.get("enabled", False):
        for ev in mag_cfg.get("events", []):
            if _in_event(t, ev):
                mag = mag + np.array(ev["disturbance_uT"], dtype=float)

    return gyro, accel, mag


def _in_event(t: float, event: dict) -> bool:
    onset = float(event["onset_time_s"])
    return onset <= t < onset + float(event["duration_s"])
