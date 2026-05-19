"""
File Name: ./src/ahrs/core/noise/profile.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  센서별 노이즈 파라미터 묶음 — 단위변환 + 파라미터 검증

    purpose:
        config.yaml의 사람 친화적 단위(deg/√h, ug 등)를
        내부 SI 단위(rad/√s, m/s²)로 변환하여 각 센서 모델에 전달.

    Notes:
        단위 변환 상수:
            ARW [deg/√h] → [rad/√s]:
                σ_arw_rad_sqrts = σ_arw_deg_sqrth * (π/180) / √3600
                                = σ_arw_deg_sqrth * π / (180 * 60)
            BI  [deg/h]  → [rad/s]:
                σ_bi_rad_s = σ_bi_deg_h * (π/180) / 3600
            RRW [deg/h/√h] → [rad/s/√s]:
                σ_rrw_rad_s_sqrts = σ_rrw_deg_h_sqrth * (π/180) / (3600 * √3600)
                                  = σ_rrw_deg_h_sqrth * π / (180 * 3600 * 60)
            VRW [m/s/√h] → [m/s²/√s]:
                σ_vrw_ms2_sqrts = σ_vrw_ms_sqrth / √3600
            BI accel [ug] → [m/s²]:
                σ_bi_ms2 = σ_bi_ug * 1e-6 * 9.80665
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

# 단위변환 상수
_DEG2RAD     = np.pi / 180.0
_RAD2DEG     = 180.0 / np.pi
_SQRT_3600      = 60.0          # √3600 = 60
_G_TO_MS2       = 9.80665

@dataclass
class GyroNoiseProfile:
    """자이로스코프 노이즈 파라미터 (내부 SI 단위)."""

    turn_on_bias_rad_s:       np.ndarray  # [rad/sec]
    arw_rad_sqrts:            float       # [rad/√sec]
    bi_sigma_rad_s:           float       # [rad/sec]
    rrw_rad_s_sqrts:          float       # [rad/sec/√sec]
    bi_corr_time_s:           float       # [sec]
    scale_factor_ppm:         float       # [ppm]
    g_sensitivity_rad_s_per_ms2: np.ndarray  # [rad/s per m/s²], shape (3,3)

    @classmethod
    def from_config(cls, cfg: dict) -> "GyroNoiseProfile":
        """config.yaml imu.gyroscope 섹션에서 생성."""
        bias = np.deg2rad(np.array(cfg["bias_deg_s"], dtype=float))

        arw_rad = cfg["arw_deg_per_sqrth"] * _DEG2RAD / _SQRT_3600
        bi_rad  = cfg["bias_instability_deg_h"] * _DEG2RAD / 3600.0
        rrw_rad = cfg["rrw_deg_per_h_sqrth"] * _DEG2RAD / (3600.0 * _SQRT_3600)

        g_sens_raw = np.array(cfg.get("g_sensitivity_deg_s_per_g", [0.0, 0.0, 0.0]),
                              dtype=float)
        # [deg/s per g] → [rad/s per m/s²]
        g_sens = np.diag(g_sens_raw * _DEG2RAD / _G_TO_MS2)

        return cls(
            turn_on_bias_rad_s=bias,
            arw_rad_sqrts=arw_rad,
            bi_sigma_rad_s=bi_rad,
            rrw_rad_s_sqrts=rrw_rad,
            bi_corr_time_s=float(cfg["bias_correlation_time_s"]),
            scale_factor_ppm=float(cfg.get("scale_factor_ppm", 0.0)),
            g_sensitivity_rad_s_per_ms2=g_sens,
        )

@dataclass
class AccelNoiseProfile:
    """가속도계 노이즈 파라미터 (내부 SI 단위)."""

    turn_on_bias_ms2:   np.ndarray  # [m/s²]
    vrw_ms2_sqrts:      float       # [m/s²/√s]
    bi_sigma_ms2:       float       # [m/s²]
    bi_corr_time_s:     float       # [s]
    scale_factor_ppm:   float       # [ppm]
    vre_coef:           float       # Vibration Rectification Error 계수

    @classmethod
    def from_config(cls, cfg: dict) -> "AccelNoiseProfile":
        """config.yaml imu.accelerometer 섹션에서 생성."""
        bias = np.array(cfg["bias_g"], dtype=float) * _G_TO_MS2

        vrw_ms2  = cfg["vrw_m_per_s_per_sqrth"] / _SQRT_3600
        bi_ms2   = cfg["bias_instability_ug"] * 1e-6 * _G_TO_MS2

        rrw_raw  = cfg.get("rrw_m_per_s2_per_sqrth", 0.0)

        return cls(
            turn_on_bias_ms2=bias,
            vrw_ms2_sqrts=vrw_ms2,
            bi_sigma_ms2=bi_ms2,
            bi_corr_time_s=float(cfg["bias_correlation_time_s"]),
            scale_factor_ppm=float(cfg.get("scale_factor_ppm", 0.0)),
            vre_coef=float(cfg.get("vre_coef", 0.0)),
        )

@dataclass
class MagNoiseProfile:
    """자기계 노이즈 + 왜곡 파라미터."""

    hard_iron_uT:       np.ndarray  # [uT], shape (3,)
    soft_iron_matrix:   np.ndarray  # [dimensionless], shape (3,3)
    noise_std_uT:       float       # [uT]
    bias_instability_uT: float      # [uT]
    bi_corr_time_s:     float       # [s]

    @classmethod
    def from_config(cls, cfg: dict) -> "MagNoiseProfile":
        """config.yaml imu.magnetometer 섹션에서 생성."""
        hard  = np.array(cfg["hard_iron_uT"], dtype=float)
        soft  = np.array(cfg["soft_iron_matrix"], dtype=float)

        return cls(
            hard_iron_uT=hard,
            soft_iron_matrix=soft,
            noise_std_uT=float(cfg["noise_std_uT"]),
            bias_instability_uT=float(cfg.get("bias_instability_uT", 0.0)),
            bi_corr_time_s=float(cfg.get("bias_correlation_time_s", 100.0)),
        )