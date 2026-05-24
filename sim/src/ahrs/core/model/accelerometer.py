"""
File Name: ./src/ahrs/core/model/accelerometer.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
    운동학 모델 (Kinematic Model):
        AHRS에서 가속도계의 역할은 중력 방향 기준벡터 제공.
        정지 혹은 등속 운동 시: a_body = R^T · g_world (중력만)
        가속 기동 시:          a_body = R^T · g_world + a_linear (외란)

    센서가 출력하는 측정값:
        a_meas = S · a_true + b_turnon + b_bi(t) + n_vrw(t) + f_vre(a_true)

    각 항의 의미:
        S           : 스케일 팩터 행렬
        b_turnon    : 전원 인가 시 고정 바이어스
        b_bi(t)     : Bias Instability (Gauss-Markov)
        n_vrw(t)    : Velocity Random Walk (white noise)
        f_vre(a)    : Vibration Rectification Error
                      고주파 진동이 DC 바이어스로 변환되는 2차 효과
                      드론이나 차량에서 중요, 정지 시뮬에선 작음

AHRS 맥락에서 핵심 이슈:
    선형 가속도(a_linear ≠ 0) 구간에서 가속도계를 중력 기준으로 쓰면 오차 발생.
    → Madgwick/Mahony: |a_meas| ≈ g 조건으로 업데이트 억제
    → EKF: 측정 잡음 공분산 R을 동적으로 키워서 accel update 약화

단위:
    입력/출력 모두 [m/s²]
"""

from __future__ import annotations

import numpy as np

from ahrs.core.noise.white import VRWNoise
from ahrs.core.noise.colored import GaussMarkovProcess
from ahrs.core.noise.profile import AccelNoiseProfile

class Accelerometer:
    def __init__(self, profile: AccelNoiseProfile, seed: int | None = None):
        self.profile = profile

        rng = np.random.default_rng(seed)
        seeds = rng.integers(0, 2**31, size=2)

        self._vrw = VRWNoise(
            sigma_per_sqrt_s=profile.vrw_ms2_sqrts,
            size=3,
            seed=int(seeds[0]),
        )
        self._bi = GaussMarkovProcess(
            sigma_bi=profile.bi_sigma_ms2,
            corr_time_s=profile.bi_corr_time_s,
            size=3,
            seed=int(seeds[1]),
        )

        sf = profile.scale_factor_ppm * 1e-6
        self._scale = np.eye(3) + np.diag([sf, sf, sf])

        self._rng = np.random.default_rng(seed)
        self._turn_on_bias = self._sample_turn_on_bias()

    # ── public API ────────────────────────────────────────────────────────────
    def measure(self, accel_true: np.ndarray, dt: float) -> np.ndarray:
        """
        진짜 비력(specific force)을 받아 측정값 반환.

        Args:
            accel_true: body frame 비력 [m/s²]
                        = R^T·g - a_linear  (중력 - 선형가속도)
                        NED에서 정지 시 ≈ [0, 0, -9.8]
            dt:         샘플 간격 [s]

        Returns:
            accel_meas: 노이즈 섞인 가속도계 출력 [m/s²]
        """
        # 1. 스케일 팩터
        a = self._scale @ accel_true

        # 2. Turn-on bias
        a = a + self._turn_on_bias

        # 3. Bias Instability
        a = a + self._bi.step(dt)

        # 4. VRW (white noise)
        a = a + self._vrw.sample(dt)

        # 5. Vibration Rectification Error
        #    |a|² 에 비례하는 DC 바이어스 (고주파 진동 환경에서만 의미 있음)
        if self.profile.vre_coef > 0:
            a = a + self._compute_vre(accel_true)

        return a

    def reset(self) -> None:
        self._bi.reset()
        self._turn_on_bias = self._sample_turn_on_bias()

    @staticmethod
    def is_dynamic(
        accel_meas: np.ndarray,
        gravity: float = 9.80665,
        threshold: float = 0.2,
    ) -> bool:
        """
        선형 가속도 외란 감지 유틸리티.

        필터 내부 또는 분석 코드에서 직접 호출용.
        (IMUMeasurement에 포함되지 않음 — 필터가 직접 판단)
        """
        return abs(np.linalg.norm(accel_meas) - gravity) > threshold * gravity

    # ── private ───────────────────────────────────────────────────────────────

    def _sample_turn_on_bias(self) -> np.ndarray:
        std = np.abs(self.profile.turn_on_bias_ms2)
        if np.all(std == 0):
            return self.profile.turn_on_bias_ms2.copy()
        return self._rng.normal(loc=self.profile.turn_on_bias_ms2, scale=std * 0.1)

    def _compute_vre(self, accel: np.ndarray) -> np.ndarray:
        """
        Vibration Rectification Error.
        vre = k · |a|² · sign(a)  (간략화된 모델)
        """
        mag_sq = np.dot(accel, accel)
        return self.profile.vre_coef * mag_sq * np.sign(accel)

    @classmethod
    def from_config(cls, cfg: dict, seed: int | None = None) -> "Accelerometer":
        profile = AccelNoiseProfile.from_config(cfg)
        return cls(profile=profile, seed=seed)