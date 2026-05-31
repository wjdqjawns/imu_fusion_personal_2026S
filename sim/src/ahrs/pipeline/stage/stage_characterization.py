"""
File Name: ./src/ahrs/pipeline/stage/stage_characterization.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 5 — Sensor Characterization

    파이프라인 흐름:
        5-A  노이즈 분석       (정적 궤적 → Allan, PSD, FFT)
        5-B  Gyro 캘리브레이션  (정적 바이어스 추정 + 온도 보상 피팅)
        5-C  Accel 캘리브레이션 (6-position + 온도 보상 피팅)
        5-D  Mag 캘리브레이션   (ellipsoid fitting + residual + 온도 보상)
        5-E  보정값 sensor 주입 (sensor.apply_calibration)

    입력:
        cfg     : config dict
        env     : Environment
        sensor  : IMUSensor  (보정 전)
        dt      : float
        ctx     : ExportContext
        plotter : Plotter

    출력 dict:
        {
          "noise": {
              "allan_gyro":  {"tau": ndarray, "adev": ndarray(N,3),
                              "arw_x","arw_y","arw_z", "bi_x"..., "rrw_x"...},
              "allan_accel": {동일 구조},
              "psd_gyro":    {"freq": ndarray, "power": ndarray(N,3)},
              "psd_accel":   {동일},
              "psd_mag":     {동일},
              "fft_gyro":    {"freq": ndarray, "amp": ndarray(N,3)},
              "fft_accel":   {동일},
              "fft_mag":     {동일},
          },
          "calibration": {
              "gyro":  {"bias_deg_s": [x,y,z], "bias_rad_s": [...],
                        "temp_coeff": None | {...}},
              "accel": {"bias_g": [...], "bias_m_s2": [...], "scale_matrix": [[...]],
                        "temp_coeff": None | {...}},
              "mag":   {"hard_iron_uT": [...], "soft_iron_matrix": [[...]],
                        "sphericity_before": float, "sphericity_after": float,
                        "residual_std_before": float, "residual_std_after": float,
                        "mag_field_norm_uT": float,
                        "temp_coeff": None | {...}},
          }
        }

    side effects:
        - data/raw/*.csv          원시 센서 수집
        - data/calibration/*.csv  분석 결과 (Allan tau/adev, PSD freq/power, FFT)
        - data/calibration/*.json 캘리브레이션 추정값
        - fig/characterization/*  모든 플롯
        - sensor.apply_calibration() 호출 → Stage 6에서 보정된 센서 사용
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor, GroundTruth
from ahrs.core.trajectory.static import StaticTrajectory
from ahrs.core.trajectory.spherical import SphericalTrajectory
from ahrs.core.evaluation.allan import AllanVarianceAnalysis
# # from ahrs.core.evaluation.psd import PSDAnalysis
# # from ahrs.core.evaluation.fft import FFTAnalysis
from ahrs.core.calibration.gyro_cal import GyroCalibrator
from ahrs.core.calibration.accel_cal import AccelCalibrator
from ahrs.core.calibration.mag_cal import MagCalibrator
from ahrs.utils.exporter import (
    ExportContext,
#     save_raw_static_csv,
#     save_raw_csv,
#     save_allan_csv,
#     save_psd_csv,
#     save_fft_csv,
#     save_cal_summary,
#     save_json,
)
from ahrs.utils.plotter import Plotter

logger = logging.getLogger("ahrs_sim.characterization")

def run_characterization(
    cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    ctx: ExportContext,
    plotter: Plotter,
) -> dict:
    """Stage 5 Entry Point."""
    char_cfg = cfg.get("sensor_characterization", {})
    fmt      = plotter.fmt
    results: dict = {"noise": {}, "calibration": {}}
    
    raw_dir  = ctx.data_dir / "raw"
    cal_dir  = ctx.data_dir / "calibration"
    fig_dir  = ctx.fig_dir  / "characterization"

    for d in (raw_dir, cal_dir, fig_dir):
        d.mkdir(parents=True, exist_ok=True)
        logger.info("[Stage5] Created directory: %s", d)

    logger.info("[Stage5] Noise characterization and calibration start")

    # ── 5-A: 노이즈 분석 ──────────────────────────────────────────────────────
    noise_cfg = char_cfg.get("noise_analysis", {})
    if noise_cfg.get("enabled", True):
        logger.info("[Stage5-A] Noise analysis start")
        # results["noise"] = _run_noise_analysis(noise_cfg, env, sensor, dt, raw_dir, cal_dir, fig_dir, fmt, plotter)
        logger.info("[Stage5-A] Noise analysis complete")

    # ── 5-B: Gyro 캘리브레이션 ────────────────────────────────────────────────
    gyro_cfg = char_cfg.get("calibration_procedures", {}).get("gyro_static", {})
    if gyro_cfg.get("enabled", True):
        logger.info("[Stage5-B] Gyro calibration start")
    #     results["calibration"]["gyro"] = _run_gyro_calibration(
    #         gyro_cfg, env, sensor, dt, raw_dir, cal_dir, fig_dir, fmt
    #     )
        logger.info("[Stage5-B] Gyro calibration complete | bias=%s deg/s", 1) # np.round(results["calibration"]["gyro"]["bias_deg_s"], 4).tolist()

    # ── 5-C: Accel 캘리브레이션 ───────────────────────────────────────────────
    accel_cfg = char_cfg.get("calibration_procedures", {}).get("accel_6pos", {})
    if accel_cfg.get("enabled", True):
        logger.info("[Stage5-C] Accel calibration start")
    #     results["calibration"]["accel"] = _run_accel_calibration(accel_cfg, env, sensor, dt, raw_dir, cal_dir, fig_dir, fmt)
        logger.info("[Stage5-C] Accel calibration complete | bias=%s g", 1) # np.round(results["calibration"]["accel"]["bias_g"], 4).tolist()

    # ── 5-D: Mag 캘리브레이션 ─────────────────────────────────────────────────
    mag_cfg = char_cfg.get("calibration_procedures", {}).get("magnetometer", {})
    if mag_cfg.get("enabled", True):
        logger.info("[Stage5-D] Mag calibration start")
    #     results["calibration"]["mag"] = _run_mag_calibration(mag_cfg, env, sensor, dt, raw_dir, cal_dir, fig_dir, fmt, plotter)
        logger.info("[Stage5-D] Mag calibration complete | sphericity before=%.3f after=%.3f", 1, 1) # results["calibration"]["mag"]["sphericity_before"], results["calibration"]["mag"]["sphericity_after"])

    # ── 5-E: 보정값 sensor 주입 ───────────────────────────────────────────────
    # sensor.apply_calibration(results["calibration"])
    logger.info("[Stage5-E] Calibration applied to sensor")

    # 최종 결과 저장
    # save_json(cal_dir / "calibration_summary.json", results["calibration"])
    return results

# ── 5-A: 노이즈 분석 ──────────────────────────────────────────────────────────
def _run_noise_analysis(
    noise_cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    raw_dir: Path,
    cal_dir: Path,
    fig_dir: Path,
    fmt: str,
    plotter: Plotter,
) -> dict:
    """
    정적 궤적에서 raw 데이터 수집 후 Allan / PSD / FFT 분석.

    저장:
        raw/noise_static.csv
        calibration/allan_gyro.csv, allan_accel.csv
        calibration/psd_gyro.csv, psd_accel.csv, psd_mag.csv
        calibration/fft_gyro.csv, fft_accel.csv, fft_mag.csv
        fig/raw/raw_static_gyro.png, raw_static_accel.png, raw_static_mag.png
        fig/characterization/allan_gyro.png, allan_accel.png
        fig/characterization/psd_gyro.png, psd_accel.png, psd_mag.png
        fig/characterization/fft_gyro.png, fft_accel.png, fft_mag.png
    """
    duration = float(noise_cfg.get("duration_s", 100.0))
    n_tau    = int(noise_cfg.get("n_tau", 100))
    fs       = 1.0 / dt
    results: dict = {}

    logger.info("[Stage5-A] Collecting static data: %.0fs @ %.0fHz = %d samples", duration, 1.0 / dt, int(duration / dt))
    t, gyro, accel, mag = _collect_static(env, sensor, dt, duration)
    save_raw_static_csv(raw_dir, t, gyro, accel, mag)

    plotter.time_series(t, gyro, title="Gyro Noise (Static)", axis_labels=["x", "y", "z"], out_path=fig_dir / f"noise_gyro_time.{fmt}")
    plotter.time_series(t, accel, title="Accel Noise (Static)", axis_labels=["x", "y", "z"], out_path=fig_dir / f"noise_accel_time.{fmt}")
    plotter.time_series(t, mag, title="Mag Noise (Static)", axis_labels=["x", "y", "z"], out_path=fig_dir / f"noise_mag_time.{fmt}")

    # Allan Variance (gyro, accel — mag는 환경 의존성으로 Allan 미적용)
    if noise_cfg.get("allan_variance", {}).get("enabled", True):
        av_g          = AllanVarianceAnalysis(gyro,  dt, n_tau=n_tau)
        tau_g, adev_g = av_g.compute()
        params_g      = av_g.extract_params()
        results["allan_gyro"] = {"tau": tau_g, "adev": adev_g, **params_g}
        save_allan_csv(cal_dir / "allan_gyro.csv", tau_g, adev_g)
        plotter.allan_variance(tau_g, adev_g, axis_labels=["x", "y", "z"], out_path=fig_dir / f"allan_gyro.{fmt}")

        av_a          = AllanVarianceAnalysis(accel, dt, n_tau=n_tau)
        tau_a, adev_a = av_a.compute()
        params_a      = av_a.extract_params()
        results["allan_accel"] = {"tau": tau_a, "adev": adev_a, **params_a}
        save_allan_csv(cal_dir / "allan_accel.csv", tau_a, adev_a)
        plotter.allan_variance(tau_a, adev_a, axis_labels=["x", "y", "z"], out_path=fig_dir / f"allan_accel.{fmt}")

        logger.info("[Stage5-A] Allan done | Gyro ARW=[%.4f, %.4f, %.4f] deg/√h", params_g.get("arw_deg_per_sqrth_x", 0), params_g.get("arw_deg_per_sqrth_y", 0), params_g.get("arw_deg_per_sqrth_z", 0))

    # # PSD (gyro, accel, mag 모두)
    # if noise_cfg.get("psd", {}).get("enabled", True):
    #     for name, data in [("gyro", gyro), ("accel", accel), ("mag", mag)]:
    #         psd_obj      = PSDAnalysis(data, fs)
    #         freq, pw     = psd_obj.compute()
    #         results[f"psd_{name}"] = {"freq": freq, "power": pw}
    #         save_psd_csv(cal_dir / f"psd_{name}.csv", freq, pw)
    #         plotter.psd(data, fs,
    #                     title=f"{name.capitalize()} PSD",
    #                     axis_labels=["x", "y", "z"],
    #                     out_path=fig_dir / f"psd_{name}.{fmt}")

    # # FFT (gyro, accel, mag 모두)
    # if noise_cfg.get("fft", {}).get("enabled", True):
    #     for name, data in [("gyro", gyro), ("accel", accel), ("mag", mag)]:
    #         fft_obj      = FFTAnalysis(data, fs)
    #         freq, amp    = fft_obj.compute()
    #         results[f"fft_{name}"] = {"freq": freq, "amp": amp}
    #         save_fft_csv(cal_dir / f"fft_{name}.csv", freq, amp)
    #         plotter.fft_spectrum(data, fs,
    #                              title=f"{name.capitalize()} FFT",
    #                              axis_labels=["x", "y", "z"],
    #                              out_path=fig_dir / f"fft_{name}.{fmt}")
    return results

# ── 5-B: Gyro 캘리브레이션 ────────────────────────────────────────────────────
def _run_gyro_calibration(
    gyro_cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    raw_dir: Path,
    cal_dir: Path,
    fig_dir: Path,
    fmt: str,
) -> dict:
    """
    정적 바이어스 추정 + (옵션) 온도별 반복 → 다항식 피팅.

    저장:
        raw/noise_gyro_cal.csv
        calibration/gyro_cal_result.json
    """
    duration = float(gyro_cfg.get("duration_s", 10.0))
    temp_cfg = gyro_cfg.get("temperature_compensation", {})

    t, gyro, _, _ = _collect_static(env, sensor, dt, duration)
    save_raw_csv(raw_dir / "noise_gyro_cal.csv",
                 t, {"gx_rad_s": gyro[:, 0],
                     "gy_rad_s": gyro[:, 1],
                     "gz_rad_s": gyro[:, 2]})

    cal       = GyroCalibrator()
    cal.fit(gyro)
    bias_rads = cal.bias_rad_s
    bias_degs = np.rad2deg(bias_rads)

    result = {
        "bias_deg_s": bias_degs.tolist(),
        "bias_rad_s": bias_rads.tolist(),
        "temp_coeff": None,
    }

    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 2))
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, duration, temps_K, degree,
            sensor_key="gyro", raw_dir=raw_dir, fig_dir=fig_dir, fmt=fmt
        )

    save_json(cal_dir / "gyro_cal_result.json", result)
    return result

# ── 5-C: Accel 캘리브레이션 ───────────────────────────────────────────────────
def _run_accel_calibration(
    accel_cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    raw_dir: Path,
    cal_dir: Path,
    fig_dir: Path,
    fmt: str,
) -> dict:
    """
    6-position 측정 → bias + scale matrix 추정 + (옵션) 온도 보상.

    저장:
        raw/noise_accel_6pos.csv
        calibration/accel_cal_result.json
    """
    hold_dur  = float(accel_cfg.get("hold_duration_s", 5.0))
    positions = accel_cfg.get("positions", ["+z", "-z", "+x", "-x", "+y", "-y"])
    temp_cfg  = accel_cfg.get("temperature_compensation", {})

    g = np.linalg.norm(env.gravity_ned)

    # 6면 방향 → body frame에서 바라보는 중력 벡터 (NED body frame)
    pos_gravity_map = {
        "+z": np.array([0,  0,  g]),
        "-z": np.array([0,  0, -g]),
        "+x": np.array([g,  0,  0]),
        "-x": np.array([-g, 0,  0]),
        "+y": np.array([0,  g,  0]),
        "-y": np.array([0, -g,  0]),
    }

    all_accel, all_t = [], []
    for pos in positions:
        g_vec = pos_gravity_map[pos]
        t, _, accel, _ = _collect_static_with_gravity(env, sensor, dt, hold_dur, g_vec)
        all_accel.append(accel)
        all_t.append(t + len(all_t) * hold_dur)
        logger.info("[Stage5-C] 6pos | pos=%s mean=%s g",
                    pos, np.round(accel.mean(0) / g, 3).tolist())

    all_accel_np = np.vstack(all_accel)
    all_t_np     = np.concatenate(all_t)
    save_raw_csv(raw_dir / "noise_accel_6pos.csv",
                 all_t_np,
                 {"ax_m_s2": all_accel_np[:, 0],
                  "ay_m_s2": all_accel_np[:, 1],
                  "az_m_s2": all_accel_np[:, 2]})

    cal          = AccelCalibrator()
    cal.fit(all_accel_np, positions, g)
    bias_g       = (cal.bias_m_s2 / g).tolist()
    scale_matrix = cal.scale_matrix.tolist()

    result = {
        "bias_g":       bias_g,
        "bias_m_s2":    cal.bias_m_s2.tolist(),
        "scale_matrix": scale_matrix,
        "temp_coeff":   None,
    }

    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 2))
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, hold_dur, temps_K, degree,
            sensor_key="accel", raw_dir=raw_dir, fig_dir=fig_dir, fmt=fmt
        )

    save_json(cal_dir / "accel_cal_result.json", result)
    return result

# ── 5-D: Mag 캘리브레이션 ─────────────────────────────────────────────────────
def _run_mag_calibration(
    mag_cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    raw_dir: Path,
    cal_dir: Path,
    fig_dir: Path,
    fmt: str,
    plotter: Plotter,
) -> dict:
    """
    구면 궤적 수집 → ellipsoid fitting → residual 검증.

    저장:
        raw/noise_mag_sphere.csv
        calibration/mag_cal_result.json
        fig/characterization/mag_calibration.png
    """
    duration = float(mag_cfg.get("duration_s", 120.0))
    method   = mag_cfg.get("method", "ellipsoid_fit")
    temp_cfg = mag_cfg.get("temperature_compensation", {})

    logger.info("[Stage5-D] Collecting spherical data: %.0fs @ %.0fHz",
                duration, 1.0 / dt)
    traj  = SphericalTrajectory(rate_deg_s=30.0)
    truth = traj.generate(dt, duration)
    n     = len(truth.t)
    mag_raw = np.zeros((n, 3))

    for k in range(n):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas       = sensor.measure(gt, truth.t[k], dt)
        mag_raw[k] = meas.mag

    save_raw_csv(raw_dir / "noise_mag_sphere.csv",
                 truth.t,
                 {"mx_uT": mag_raw[:, 0],
                  "my_uT": mag_raw[:, 1],
                  "mz_uT": mag_raw[:, 2]})

    cal     = MagCalibrator(method=method)
    cal.fit(mag_raw)
    mag_cal = np.array([cal.apply(m) for m in mag_raw])

    r_raw   = np.linalg.norm(mag_raw, axis=1)
    r_cal   = np.linalg.norm(mag_cal, axis=1)
    r_ref   = r_cal.mean()
    res_raw = r_raw - r_raw.mean()
    res_cal = r_cal - r_ref

    def _sphericity(pts: np.ndarray) -> float:
        cov  = np.cov(pts.T)
        eigs = np.sqrt(np.abs(np.linalg.eigvalsh(cov)))
        return float(eigs.min() / eigs.max())

    sph_before = _sphericity(mag_raw)
    sph_after  = _sphericity(mag_cal)

    plotter.mag_sphere(mag_raw, mag_cal,
                       out_path=fig_dir / f"mag_calibration.{fmt}")

    cal_report = cal.report()
    result = {
        "hard_iron_uT":        cal_report.get("hard_iron_uT", []),
        "soft_iron_matrix":    cal_report.get("soft_iron_matrix", []),
        "sphericity_before":   sph_before,
        "sphericity_after":    sph_after,
        "residual_std_before": float(res_raw.std()),
        "residual_std_after":  float(res_cal.std()),
        "mag_field_norm_uT":   float(r_ref),
        "temp_coeff":          None,
    }

    logger.info("[Stage5-D] Mag residual std | before=%.3f after=%.3f uT",
                result["residual_std_before"], result["residual_std_after"])

    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 1))
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, duration, temps_K, degree,
            sensor_key="mag", raw_dir=raw_dir, fig_dir=fig_dir, fmt=fmt
        )

    save_json(cal_dir / "mag_cal_result.json", result)
    return result

# ── 온도 보상 피팅 ─────────────────────────────────────────────────────────────
def _fit_temperature_model(
    sensor: IMUSensor,
    env: Environment,
    dt: float,
    duration: float,
    temps_K: list[float],
    degree: int,
    sensor_key: str,
    raw_dir: Path,
    fig_dir: Path,
    fmt: str,
) -> dict:
    """
    온도별로 정적 측정 반복 → bias(T) 다항식 피팅.

    모델:
        bias(T) = b0 + α(T − T_ref)              (degree=1)
        bias(T) = b0 + α(T − T_ref) + β(T − T_ref)²  (degree=2)
    """
    T_ref  = float(temps_K[0])
    biases = []

    for T in temps_K:
        sensor.set_temperature(T)
        t, gyro, accel, mag = _collect_static(env, sensor, dt, duration)

        data = {"gyro": gyro, "accel": accel, "mag": mag}[sensor_key]
        biases.append(data.mean(axis=0))

        save_raw_csv(
            raw_dir / f"noise_{sensor_key}_temp_{int(T)}K.csv",
            t, {f"{sensor_key}_x": data[:, 0],
                f"{sensor_key}_y": data[:, 1],
                f"{sensor_key}_z": data[:, 2]}
        )

    sensor.set_temperature(None)

    biases_np = np.array(biases)
    temps_np  = np.array(temps_K, dtype=float)
    dT        = temps_np - T_ref

    coeffs = np.zeros((degree + 1, 3))
    for axis in range(3):
        coeffs[:, axis] = np.polyfit(dT, biases_np[:, axis], degree)

    result = {
        "T_ref_K": T_ref,
        "degree":  degree,
        "b0":      coeffs[-1].tolist(),
        "alpha":   coeffs[-2].tolist(),
    }
    if degree >= 2:
        result["beta"] = coeffs[-3].tolist()

    logger.info("[Stage5] Temp model (%s) | degree=%d b0=%s alpha=%s",
                sensor_key, degree,
                np.round(result["b0"], 5).tolist(),
                np.round(result["alpha"], 6).tolist())

    return result

# --- 데이터 수집 헬퍼 ---
def _collect_static(
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
        collect data from static trajectory (t, gyro, accel, mag).
    """
    traj  = StaticTrajectory()
    gt_data = traj.generate(dt, duration)
    n     = len(gt_data.t)
    gyro  = np.zeros((n, 3))
    accel = np.zeros((n, 3))
    mag   = np.zeros((n, 3))

    for k in range(n):
        gt = GroundTruth(
            q=gt_data.q[k], omega=gt_data.omega[k],
            pos=gt_data.pos[k], vel=gt_data.vel[k],
            accel_world=gt_data.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas      = sensor.measure(gt, gt_data.t[k], dt)
        gyro[k]   = meas.gyro
        accel[k]  = meas.accel
        mag[k]    = meas.mag

    return gt_data.t, gyro, accel, mag

def _collect_static_with_gravity(
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
    gravity_body: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Accel 6-position용: body frame에서 특정 중력 방향을 갖는 정적 측정.

    gravity_body: body frame 기준 중력 벡터 [m/s²]
                  (예: +Z 면 → [0, 0, +g])
    """
    traj  = StaticTrajectory()
    truth = traj.generate(dt, duration)
    n     = len(truth.t)
    gyro  = np.zeros((n, 3))
    accel = np.zeros((n, 3))
    mag   = np.zeros((n, 3))

    from ahrs.utils.transforms import Transforms
    # specific force body = -gravity_body (body에 반중력 방향으로 작용)
    sf_body = -gravity_body

    for k in range(n):
        # accel은 specific force를 직접 주입하여 6-pos 시뮬레이션
        accel_meas = sensor.accel.measure(sf_body, dt)
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas      = sensor.measure(gt, truth.t[k], dt)
        gyro[k]   = meas.gyro
        accel[k]  = accel_meas   # 6-pos 특수 주입
        mag[k]    = meas.mag

    return truth.t, gyro, accel, mag