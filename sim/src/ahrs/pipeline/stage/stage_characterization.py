"""
File Name: ./src/ahrs/pipeline/stage_characterization.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Stage 5 — Sensor Characterization

    파이프라인 흐름:
        5-A  노이즈 분석      (정적 궤적 → Allan, PSD, FFT)
        5-B  Gyro 캘리브레이션 (정적 바이어스 추정 + 온도 보상 피팅)
        5-C  Accel 캘리브레이션 (6-position + 온도 보상 피팅)
        5-D  Mag 캘리브레이션  (ellipsoid fitting + residual 검증 + 온도 보상)
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
                              "arw": [x,y,z], "bi": [...], "rrw": [...]},
              "allan_accel": {...},
              "psd_gyro":    {"freq": ndarray, "power": ndarray(N,3)},
              "psd_accel":   {...},
              "psd_mag":     {...},
              "fft_gyro":    {"freq": ndarray, "amp": ndarray(N,3)},
              "fft_accel":   {...},
              "fft_mag":     {...},
          },
          "calibration": {
              "gyro":  {"bias_deg_s": [x,y,z],
                        "temp_coeff": {"b0": [...], "alpha": [...], "beta": [...]}},
              "accel": {"bias_g": [x,y,z], "scale_matrix": [[...]],
                        "temp_coeff": {"b0": [...], "alpha": [...], "beta": [...]}},
              "mag":   {"hard_iron_uT": [x,y,z],
                        "soft_iron_matrix": [[...]],
                        "sphericity_before": float, "sphericity_after": float,
                        "residual_std_before": float, "residual_std_after": float,
                        "temp_coeff": {"b0": [...], "alpha": [...]}},
          }
        }

    side effects:
        - data/raw/*.csv          원시 센서 데이터
        - data/calibration/*.csv  분석 결과 (Allan tau/adev, PSD freq/power, FFT)
        - data/calibration/*.json 캘리브레이션 추정값
        - fig/characterization/*  모든 플롯
        - sensor.apply_calibration() 호출 → Stage 6에서 보정된 센서 사용
"""

from __future__ import annotations

import csv
import json
import logging
from pathlib import Path

import numpy as np

from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor, GroundTruth
from ahrs.core.trajectory.static import StaticTrajectory
from ahrs.core.trajectory.spherical import SphericalTrajectory
from ahrs.core.evaluation.allan import AllanVariance
from ahrs.core.evaluation.psd import PSD
from ahrs.core.evaluation.fft import FFTAnalysis
from ahrs.core.calibration.gyro_cal import GyroCalibrator
from ahrs.core.calibration.accel_cal import AccelCalibrator
from ahrs.core.calibration.mag_cal import MagCalibrator

logger = logging.getLogger("ahrs_sim.characterization")

def run_characterization(
    cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    ctx,           # ExportContext
    plotter,       # Plotter
) -> dict:
    """
    Stage 5 전체 실행.
    반환값은 runner가 report stage에 전달하고,
    sensor.apply_calibration()으로 Stage 6 이전에 보정값을 주입.
    """
    char_cfg = cfg.get("sensor_characterization", {})
    fmt      = plotter.fmt

    # 출력 디렉터리
    raw_dir  = ctx.data_dir / "raw"
    cal_dir  = ctx.data_dir / "calibration"
    fig_dir  = ctx.fig_dir  / "characterization"
    for d in (raw_dir, cal_dir, fig_dir):
        d.mkdir(parents=True, exist_ok=True)

    results: dict = {"noise": {}, "calibration": {}}

    # ── 5-A: 노이즈 분석 ──────────────────────────────────────────────────────
    if char_cfg.get("noise_analysis", {}).get("enabled", True):
        logger.info("[Stage5-A] Noise analysis start")
        results["noise"] = _run_noise_analysis(
            char_cfg["noise_analysis"], env, sensor, dt,
            raw_dir, cal_dir, fig_dir, fmt
        )
        logger.info("[Stage5-A] Noise analysis complete")

    # ── 5-B: Gyro 캘리브레이션 ────────────────────────────────────────────────
    gyro_cal_cfg = char_cfg.get("calibration_procedures", {}).get("gyro_static", {})
    if gyro_cal_cfg.get("enabled", True):
        logger.info("[Stage5-B] Gyro calibration start")
        results["calibration"]["gyro"] = _run_gyro_calibration(
            gyro_cal_cfg, env, sensor, dt,
            raw_dir, cal_dir, fig_dir, fmt
        )
        logger.info("[Stage5-B] Gyro calibration complete | bias=%s deg/s",
                    np.round(results["calibration"]["gyro"]["bias_deg_s"], 4).tolist())

    # ── 5-C: Accel 캘리브레이션 ───────────────────────────────────────────────
    accel_cal_cfg = char_cfg.get("calibration_procedures", {}).get("accel_6pos", {})
    if accel_cal_cfg.get("enabled", False):
        logger.info("[Stage5-C] Accel calibration start")
        results["calibration"]["accel"] = _run_accel_calibration(
            accel_cal_cfg, env, sensor, dt,
            raw_dir, cal_dir, fig_dir, fmt
        )
        logger.info("[Stage5-C] Accel calibration complete | bias=%s g",
                    np.round(results["calibration"]["accel"]["bias_g"], 4).tolist())

    # ── 5-D: Mag 캘리브레이션 ─────────────────────────────────────────────────
    mag_cal_cfg = char_cfg.get("calibration_procedures", {}).get("magnetometer", {})
    if mag_cal_cfg.get("enabled", True):
        logger.info("[Stage5-D] Mag calibration start")
        results["calibration"]["mag"] = _run_mag_calibration(
            mag_cal_cfg, env, sensor, dt,
            raw_dir, cal_dir, fig_dir, fmt, plotter
        )
        logger.info("[Stage5-D] Mag calibration complete | "
                    "sphericity before=%.3f after=%.3f",
                    results["calibration"]["mag"]["sphericity_before"],
                    results["calibration"]["mag"]["sphericity_after"])

    # ── 5-E: 보정값 sensor 주입 ───────────────────────────────────────────────
    # Stage 6 estimation에서 보정된 sensor 사용
    sensor.apply_calibration(results["calibration"])
    logger.info("[Stage5-E] Calibration applied to sensor")

    # 캘리브레이션 결과 JSON 저장
    _save_cal_summary(cal_dir, results["calibration"])

    return results

def _run_noise_analysis(
    noise_cfg: dict,
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    raw_dir: Path,
    cal_dir: Path,
    fig_dir: Path,
    fmt: str,
) -> dict:
    """
    정적 궤적에서 raw 데이터 수집 후 Allan / PSD / FFT 분석.

    저장:
        raw/noise_static.csv
        calibration/allan_gyro.csv, allan_accel.csv
        calibration/psd_gyro.csv, psd_accel.csv, psd_mag.csv
        calibration/fft_gyro.csv, fft_accel.csv, fft_mag.csv
        fig/characterization/allan_gyro.png, allan_accel.png
        fig/characterization/psd_gyro.png, psd_accel.png, psd_mag.png
        fig/characterization/fft_gyro.png, fft_accel.png, fft_mag.png
    """
    duration = float(noise_cfg.get("duration_s", 300.0))
    n_tau    = int(noise_cfg.get("n_tau", 100))

    # 정적 raw 수집
    logger.info("[Stage5-A] Collecting static data: %.0fs @ %.0fHz = %d samples",
                duration, 1/dt, int(duration/dt))
    t, gyro, accel, mag = _collect_static(env, sensor, dt, duration)
    _save_raw_static(raw_dir, t, gyro, accel, mag)

    results: dict = {}
    fs = 1.0 / dt

    # Allan Variance (gyro, accel만 — mag은 환경 의존성으로 Allan 미적용)
    if noise_cfg.get("allan_variance", {}).get("enabled", True):
        # Gyro
        av_g       = AllanVariance(gyro, dt, n_tau=n_tau)
        tau_g, adev_g = av_g.compute()
        params_g   = av_g.extract_params()
        results["allan_gyro"] = {
            "tau": tau_g, "adev": adev_g, **params_g
        }
        _save_allan_csv(cal_dir / "allan_gyro.csv", tau_g, adev_g)
        # plotter.allan_variance(tau_g, adev_g, ["x","y","z"],
        #                        out_path=fig_dir / f"allan_gyro.{fmt}")

        # Accel
        av_a       = AllanVariance(accel, dt, n_tau=n_tau)
        tau_a, adev_a = av_a.compute()
        params_a   = av_a.extract_params()
        results["allan_accel"] = {
            "tau": tau_a, "adev": adev_a, **params_a
        }
        _save_allan_csv(cal_dir / "allan_accel.csv", tau_a, adev_a)
        # plotter.allan_variance(tau_a, adev_a, ["x","y","z"],
        #                        out_path=fig_dir / f"allan_accel.{fmt}")

        logger.info("[Stage5-A] Allan done | Gyro ARW=[%.4f, %.4f, %.4f] deg/√h",
                    params_g.get("arw_x", 0), params_g.get("arw_y", 0),
                    params_g.get("arw_z", 0))

    # PSD (gyro, accel, mag 모두)
    if noise_cfg.get("psd", {}).get("enabled", True):
        for name, data in [("gyro", gyro), ("accel", accel), ("mag", mag)]:
            psd      = PSD(data, fs)
            freq, pw = psd.compute()
            results[f"psd_{name}"] = {"freq": freq, "power": pw}
            _save_psd_csv(cal_dir / f"psd_{name}.csv", freq, pw)
            # plotter.psd(data, fs, title=f"{name.capitalize()} PSD",
            #             axis_labels=["x","y","z"],
            #             out_path=fig_dir / f"psd_{name}.{fmt}")

    # FFT (gyro, accel, mag 모두)
    if noise_cfg.get("fft", {}).get("enabled", True):
        for name, data in [("gyro", gyro), ("accel", accel), ("mag", mag)]:
            fft_a    = FFTAnalysis(data, fs)
            freq, amp = fft_a.compute()
            results[f"fft_{name}"] = {"freq": freq, "amp": amp}
            _save_fft_csv(cal_dir / f"fft_{name}.csv", freq, amp)
            # plotter.fft_spectrum(data, fs,
            #                      out_path=fig_dir / f"fft_{name}.{fmt}")

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
        raw/noise_gyro_cal.csv          정적 수집 raw
        raw/noise_gyro_temp_{T}K.csv    온도별 수집 raw (온도 보상 시)
        calibration/gyro_cal_result.json
        fig/characterization/gyro_bias_static.png
        fig/characterization/temp_compensation_gyro.png (온도 보상 시)
    """
    duration = float(gyro_cfg.get("duration_s", 10.0))
    temp_cfg = gyro_cfg.get("temperature_compensation", {})

    # 기준 온도에서 정적 수집
    t, gyro, _, _ = _collect_static(env, sensor, dt, duration)
    _save_raw_csv(raw_dir / "noise_gyro_cal.csv",
                  t, {"gx_rad_s": gyro[:,0], "gy_rad_s": gyro[:,1], "gz_rad_s": gyro[:,2]})

    # 바이어스 추정
    cal      = GyroCalibrator()
    cal.fit(gyro)
    bias_rads = cal.bias_rad_s          # ndarray (3,)
    bias_degs = np.rad2deg(bias_rads)

    # plotter.gyro_bias_timeseries(t, gyro, bias_rads,
    #                              out_path=fig_dir / f"gyro_bias_static.{fmt}")

    result = {
        "bias_deg_s":  bias_degs.tolist(),
        "bias_rad_s":  bias_rads.tolist(),
        "temp_coeff":  None,
    }

    # 온도 보상 피팅
    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 2))
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, duration, temps_K, degree,
            sensor_key="gyro", raw_dir=raw_dir,
            fig_dir=fig_dir, fmt=fmt
        )
        logger.info("[Stage5-B] Gyro temp coeff fitted | degree=%d temps=%s",
                    degree, temps_K)

    _save_json(cal_dir / "gyro_cal_result.json", result)
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

    6면 측정 시뮬레이션:
        각 면을 고정된 자세로 hold_duration_s 동안 측정.
        각 면에서 mean(accel)이 해당 중력 방향이어야 함.

    저장:
        raw/noise_accel_6pos.csv
        calibration/accel_cal_result.json
        fig/characterization/accel_6pos.png
    """
    hold_dur  = float(accel_cfg.get("hold_duration_s", 5.0))
    positions = accel_cfg.get("positions", ["+z", "-z", "+x", "-x", "+y", "-y"])
    temp_cfg  = accel_cfg.get("temperature_compensation", {})

    g = np.linalg.norm(env.gravity_ned)

    # 6면 방향 → 중력벡터 매핑 (NED frame)
    pos_gravity_map = {
        "+z": np.array([0, 0,  g]),   # face down → gravity body +Z
        "-z": np.array([0, 0, -g]),
        "+x": np.array([ g, 0, 0]),
        "-x": np.array([-g, 0, 0]),
        "+y": np.array([0,  g, 0]),
        "-y": np.array([0, -g, 0]),
    }

    all_accel = []
    all_t     = []
    for pos in positions:
        g_vec = pos_gravity_map[pos]
        t, _, accel, _ = _collect_static_with_gravity(env, sensor, dt, hold_dur, g_vec)
        all_accel.append(accel)
        all_t.append(t + (len(all_t) * hold_dur))
        logger.info("[Stage5-C] 6pos | pos=%s mean=%s g",
                    pos, np.round(accel.mean(0) / g, 3).tolist())

    all_accel_np = np.vstack(all_accel)
    all_t_np     = np.concatenate(all_t)
    _save_raw_csv(raw_dir / "noise_accel_6pos.csv",
                  all_t_np,
                  {"ax_m_s2": all_accel_np[:,0],
                   "ay_m_s2": all_accel_np[:,1],
                   "az_m_s2": all_accel_np[:,2]})

    # 캘리브레이션 추정
    cal = AccelCalibrator()
    cal.fit(all_accel_np, positions, g)
    bias_g      = (cal.bias_m_s2 / g).tolist()
    scale_matrix = cal.scale_matrix.tolist()

    # plotter.accel_6pos(all_accel_np, positions, cal,
    #                    out_path=fig_dir / f"accel_6pos.{fmt}")

    result = {
        "bias_g":        bias_g,
        "bias_m_s2":     cal.bias_m_s2.tolist(),
        "scale_matrix":  scale_matrix,
        "temp_coeff":    None,
    }

    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 2))
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, hold_dur, temps_K, degree,
            sensor_key="accel", raw_dir=raw_dir,
            fig_dir=fig_dir, fmt=fmt
        )

    _save_json(cal_dir / "accel_cal_result.json", result)
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
    plotter,
) -> dict:
    """
    구면 궤적 수집 → ellipsoid fitting → residual 검증 + (옵션) 온도 보상.

    플롯 2장:
        mag_trajectory.png  : 점군 + 타원 (궤적 / 균일성 / 왜곡 형태)
        mag_residual.png    : residual 히스토그램 + 극좌표 + sphericity 지표

    저장:
        raw/noise_mag_sphere.csv
        calibration/mag_cal_result.json
        fig/characterization/mag_trajectory.png
        fig/characterization/mag_residual.png
    """
    duration = float(mag_cfg.get("duration_s", 120.0))
    method   = mag_cfg.get("method", "ellipsoid_fit")
    temp_cfg = mag_cfg.get("temperature_compensation", {})

    # 구면 궤적 raw 수집
    logger.info("[Stage5-D] Collecting spherical data: %.0fs @ %.0fHz",
                duration, 1/dt)
    traj = SphericalTrajectory(rate_deg_s=30.0)
    truth = traj.generate(dt, duration)
    n = len(truth.t)
    mag_raw = np.zeros((n, 3))

    for k in range(n):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=truth.accel_world[k],
            mag_world=env.mag_ned,
        )
        meas = sensor.measure(gt, truth.t[k], dt)
        mag_raw[k] = meas.mag

    _save_raw_csv(raw_dir / "noise_mag_sphere.csv",
                  truth.t,
                  {"mx_uT": mag_raw[:,0],
                   "my_uT": mag_raw[:,1],
                   "mz_uT": mag_raw[:,2]})

    # Ellipsoid fitting
    cal = MagCalibrator(method=method)
    cal.fit(mag_raw)
    mag_cal = np.array([cal.apply(m) for m in mag_raw])

    # Residual 계산
    r_raw = np.linalg.norm(mag_raw, axis=1)
    r_cal = np.linalg.norm(mag_cal, axis=1)
    r_ref = r_cal.mean()

    res_raw = r_raw - r_raw.mean()
    res_cal = r_cal - r_ref

    # Sphericity: 공분산 고유값으로 타원체 반축 비율
    def sphericity(pts: np.ndarray) -> float:
        cov  = np.cov(pts.T)
        eigs = np.sqrt(np.abs(np.linalg.eigvalsh(cov)))
        return float(eigs.min() / eigs.max())

    sph_before = sphericity(mag_raw)
    sph_after  = sphericity(mag_cal)

    # 플롯 1: 궤적 + 타원 (샘플링 균일성, 왜곡 형태)
    # plotter.mag_trajectory(mag_raw, mag_cal,
    #                        out_path=fig_dir / f"mag_trajectory.{fmt}")

    # 플롯 2: Residual 분석 (보정 품질 정량화)
    # plotter.mag_residual(res_raw, res_cal, mag_cal,
    #                      sphericity_before=sph_before,
    #                      sphericity_after=sph_after,
    #                      out_path=fig_dir / f"mag_residual.{fmt}")

    cal_report = cal.report()
    result = {
        "hard_iron_uT":       cal_report.get("hard_iron_uT", []),
        "soft_iron_matrix":   cal_report.get("soft_iron_matrix", []),
        "sphericity_before":  sph_before,
        "sphericity_after":   sph_after,
        "residual_std_before": float(res_raw.std()),
        "residual_std_after":  float(res_cal.std()),
        "mag_field_norm_uT":  float(r_ref),
        "temp_coeff":         None,
    }

    logger.info("[Stage5-D] Mag residual std | before=%.3f after=%.3f uT",
                result["residual_std_before"], result["residual_std_after"])

    if temp_cfg.get("enabled", False):
        temps_K = temp_cfg.get("temperatures_K", [273.15, 298.15, 323.15])
        degree  = int(temp_cfg.get("degree", 1))  # mag은 1차로 충분
        result["temp_coeff"] = _fit_temperature_model(
            sensor, env, dt, duration, temps_K, degree,
            sensor_key="mag", raw_dir=raw_dir,
            fig_dir=fig_dir, fmt=fmt
        )

    _save_json(cal_dir / "mag_cal_result.json", result)
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
        bias(T) = b0 + α(T - T_ref) + β(T - T_ref)²   (degree=2)
        bias(T) = b0 + α(T - T_ref)                    (degree=1)

    T_ref = 첫 번째 온도 (기준점)

    저장:
        raw/noise_{sensor_key}_temp_{T}K.csv  각 온도별 raw
        fig/characterization/temp_compensation_{sensor_key}.png
    """
    T_ref  = float(temps_K[0])
    biases = []   # (n_temps, 3)

    for T in temps_K:
        # 센서 온도 임시 설정 (thermal_field가 지원하는 경우)
        sensor.set_temperature(T)
        t, gyro, accel, mag = _collect_static(env, sensor, dt, duration)

        if sensor_key == "gyro":
            data = gyro
        elif sensor_key == "accel":
            data = accel
        else:
            data = mag

        bias_T = data.mean(axis=0)
        biases.append(bias_T)

        # 온도별 raw 저장
        _save_raw_csv(
            raw_dir / f"noise_{sensor_key}_temp_{int(T)}K.csv",
            t, {f"{sensor_key}_x": data[:,0],
                f"{sensor_key}_y": data[:,1],
                f"{sensor_key}_z": data[:,2]}
        )

    sensor.set_temperature(None)  # 기본 온도로 복원

    biases_np = np.array(biases)   # (n_temps, 3)
    temps_np  = np.array(temps_K, dtype=float)
    dT        = temps_np - T_ref

    # 다항식 피팅 (축별 독립)
    coeffs = np.zeros((degree + 1, 3))
    for axis in range(3):
        coeffs[:, axis] = np.polyfit(dT, biases_np[:, axis], degree)
        # np.polyfit 반환: [최고차 계수, ..., 상수항]

    # 계수를 b0, alpha, (beta) 형태로 정리
    result = {
        "T_ref_K":   T_ref,
        "degree":    degree,
        "b0":        coeffs[-1].tolist(),       # 상수항
        "alpha":     coeffs[-2].tolist(),       # 1차 계수
    }
    if degree >= 2:
        result["beta"] = coeffs[-3].tolist()    # 2차 계수

    # plotter.temp_compensation(temps_np, biases_np, coeffs, T_ref,
    #                           sensor_name=sensor_key,
    #                           out_path=fig_dir / f"temp_compensation_{sensor_key}.{fmt}")

    logger.info("[Stage5] Temp model (%s) | degree=%d b0=%s alpha=%s",
                sensor_key, degree,
                np.round(result["b0"], 5).tolist(),
                np.round(result["alpha"], 6).tolist())

    return result


# ── 데이터 수집 헬퍼 ──────────────────────────────────────────────────────────
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

# Stage 5: Noise characterization -------------------------------------------------
def _run_noise_stage(cfg: dict, env: Environment, sensor: IMUSensor, dt: float, ctx: ExportContext, plotter: Plotter) -> dict:
    cal_cfg = cfg.get("calibration", {})
    fmt     = plotter.fmt
    results: dict = {}

    # Allan variance + PSD + FFT (static)
    try:
        av_cfg = cal_cfg.get("allan_variance", {})
        if av_cfg.get("enabled", True):
            static_dur = float(av_cfg.get("duration_s", 300.0))
            logger.info("Static: %.0fs @ %.0fHz = %d samples", static_dur, 1 / dt, int(static_dur / dt))

            t_s, gyro_s, accel_s, mag_s = _collect_raw(StaticTrajectory(), env, sensor, dt, static_dur)
            save_noise_static_csv(ctx.data_dir, t_s, gyro_s, accel_s, mag_s)

            av_g = AllanVariance(gyro_s, dt)
            tau_g, adev_g = av_g.compute()
            results["allan_gyro"] = av_g.extract_params()

            av_a = AllanVariance(accel_s, dt)
            tau_a, adev_a = av_a.compute()
            results["allan_accel"] = av_a.extract_params()

            plotter.allan_variance(tau_g, adev_g, axis_labels=["x", "y", "z"], out_path=ctx.fig_dir / f"allan_gyro.{fmt}")
            plotter.allan_variance(tau_a, adev_a, axis_labels=["x", "y", "z"], out_path=ctx.fig_dir / f"allan_accel.{fmt}")

            fs = 1.0 / dt
            plotter.psd(gyro_s,  fs, title="Gyroscope PSD", axis_labels=["x", "y", "z"], unit_label="(rad/s)²/Hz", out_path=ctx.fig_dir / f"psd_gyro.{fmt}")
            plotter.psd(accel_s, fs, title="Accelerometer PSD", axis_labels=["x", "y", "z"], unit_label="(m/s²)²/Hz", out_path=ctx.fig_dir / f"psd_accel.{fmt}")
            plotter.psd(mag_s,   fs, title="Magnetometer PSD", axis_labels=["x", "y", "z"], unit_label="µT²/Hz", out_path=ctx.fig_dir / f"psd_mag.{fmt}")
            plotter.fft_spectrum(gyro_s,  fs, title="Gyroscope FFT Spectrum", axis_labels=["x", "y", "z"], unit_label="rad/s", out_path=ctx.fig_dir / f"fft_gyro.{fmt}")
            plotter.fft_spectrum(accel_s, fs, title="Accelerometer FFT Spectrum", axis_labels=["x", "y", "z"], unit_label="m/s²", out_path=ctx.fig_dir / f"fft_accel.{fmt}")
            plotter.fft_spectrum(mag_s,   fs, title="Magnetometer FFT Spectrum", axis_labels=["x", "y", "z"], unit_label="µT", out_path=ctx.fig_dir / f"fft_mag.{fmt}")

            logger.info(r"Allan done — Gyro ARW_x=%.4f deg\sqrt{h}", results["allan_gyro"].get("arw_deg_per_sqrth_x", 0))
    except Exception as e:
        logger.error("Error occurred while computing Allan variance: %s", e)
        pass

    # ── Mag hard/soft iron (spherical) ────────────────────────────────────────
    try: 
        mag_cal_cfg = cal_cfg.get("magnetometer", {})
        if mag_cal_cfg.get("enabled", True):
            sphere_dur = float(mag_cal_cfg.get("duration_s", 120.0))
            logger.info("Spherical: %.0fs @ %.0fHz = %d samples",
                        sphere_dur, 1 / dt, int(sphere_dur / dt))

            t_sp, _, _, mag_sp = _collect_raw(
                SphericalTrajectory(rate_deg_s=30.0), env, sensor, dt, sphere_dur
            )
            save_noise_sphere_csv(ctx.data_dir, t_sp, mag_sp)

            method  = mag_cal_cfg.get("method", "ellipsoid_fit")
            mag_cal = MagCalibrator(method=method)
            mag_cal.fit(mag_sp)
            mag_sp_cal = np.array([mag_cal.apply(m) for m in mag_sp])
            results["mag_cal"] = mag_cal.report()

            plotter.mag_sphere(mag_sp, mag_sp_cal,
                            out_path=ctx.fig_dir / f"mag_calibration.{fmt}")

            raw_r = np.linalg.norm(mag_sp, axis=1)
            cal_r = np.linalg.norm(mag_sp_cal, axis=1)
            logger.info("Mag cal done — |m| raw=%.2f±%.3f  cal=%.2f±%.3f µT",
                        raw_r.mean(), raw_r.std(), cal_r.mean(), cal_r.std())
    except Exception as e:
        logger.error("Error occurred during magnetometer calibration: %s", e)
        pass

    return results

def _collect_static(
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """정적 궤적에서 (t, gyro, accel, mag) 수집."""
    traj  = StaticTrajectory()
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
        meas       = sensor.measure(gt, truth.t[k], dt)
        gyro[k]    = meas.gyro
        accel[k]   = meas.accel
        mag[k]     = meas.mag

    return truth.t, gyro, accel, mag


def _collect_static_with_gravity(
    env: Environment,
    sensor: IMUSensor,
    dt: float,
    duration: float,
    gravity_body: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Accel 6-position용: 특정 중력 방향을 body frame에서 주입하여 수집.
    gravity_body: body frame에서의 중력벡터 [m/s²]
    """
    traj  = StaticTrajectory()
    truth = traj.generate(dt, duration)
    n     = len(truth.t)
    gyro  = np.zeros((n, 3))
    accel = np.zeros((n, 3))
    mag   = np.zeros((n, 3))

    for k in range(n):
        gt = GroundTruth(
            q=truth.q[k], omega=truth.omega[k],
            pos=truth.pos[k], vel=truth.vel[k],
            accel_world=gravity_body,  # 해당 면의 중력 방향
            mag_world=env.mag_ned,
        )
        meas       = sensor.measure(gt, truth.t[k], dt)
        gyro[k]    = meas.gyro
        accel[k]   = meas.accel
        mag[k]     = meas.mag

    return truth.t, gyro, accel, mag


# ── CSV / JSON 저장 헬퍼 ──────────────────────────────────────────────────────

def _save_raw_static(
    raw_dir: Path,
    t: np.ndarray,
    gyro: np.ndarray,
    accel: np.ndarray,
    mag: np.ndarray,
) -> None:
    """정적 raw 데이터 전체를 한 파일로 저장."""
    path = raw_dir / "noise_static.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s",
                    "gx_rad_s", "gy_rad_s", "gz_rad_s",
                    "ax_m_s2",  "ay_m_s2",  "az_m_s2",
                    "mx_uT",    "my_uT",    "mz_uT"])
        for k in range(len(t)):
            w.writerow([t[k], *gyro[k], *accel[k], *mag[k]])
    logger.debug("[Stage5] Saved raw static → %s", path)


def _save_raw_csv(path: Path, t: np.ndarray, columns: dict) -> None:
    """범용 raw CSV 저장. columns = {"col_name": ndarray}"""
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + list(columns.keys()))
        for k in range(len(t)):
            row = [t[k]] + [v[k] for v in columns.values()]
            w.writerow(row)
    logger.debug("[Stage5] Saved raw → %s", path)


def _save_allan_csv(path: Path, tau: np.ndarray, adev: np.ndarray) -> None:
    """Allan deviation 결과 저장. adev: (N, 3) — x, y, z축."""
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tau_s", "adev_x", "adev_y", "adev_z"])
        for k in range(len(tau)):
            w.writerow([tau[k], *adev[k]])
    logger.debug("[Stage5] Saved Allan → %s", path)


def _save_psd_csv(path: Path, freq: np.ndarray, power: np.ndarray) -> None:
    """PSD 결과 저장. power: (N, 3)"""
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["freq_hz", "power_x", "power_y", "power_z"])
        for k in range(len(freq)):
            w.writerow([freq[k], *power[k]])
    logger.debug("[Stage5] Saved PSD → %s", path)


def _save_fft_csv(path: Path, freq: np.ndarray, amp: np.ndarray) -> None:
    """FFT 결과 저장. amp: (N, 3)"""
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["freq_hz", "amp_x", "amp_y", "amp_z"])
        for k in range(len(freq)):
            w.writerow([freq[k], *amp[k]])
    logger.debug("[Stage5] Saved FFT → %s", path)


def _save_cal_summary(cal_dir: Path, calibration: dict) -> None:
    """캘리브레이션 전체 요약을 JSON으로 저장."""
    path = cal_dir / "calibration_summary.json"
    _save_json(path, calibration)
    logger.info("[Stage5] Calibration summary → %s", path)


def _save_json(path: Path, data: dict) -> None:
    """ndarray를 list로 변환하여 JSON 저장."""
    def _convert(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, dict):
            return {k: _convert(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_convert(i) for i in obj]
        return obj

    with path.open("w", encoding="utf-8") as f:
        json.dump(_convert(data), f, indent=2, ensure_ascii=False)
    logger.debug("[Stage5] Saved JSON → %s", path)