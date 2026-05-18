"""
gyroscope.py — 자이로스코프 센서 모델

운동학 모델 (Kinematic Model):
    실제 각속도 ω_true [rad/s] 가 입력으로 들어오면
    센서가 출력하는 측정값 ω_meas 를 만들어냄.

    ω_meas = S · ω_true + b_turnon + b_bi(t) + b_rrw(t) + n_arw(t) + K_g · a

    각 항의 의미:
        S           : 스케일 팩터 행렬 (이상적이면 단위행렬)
        b_turnon    : 전원 인가 시 고정 바이어스 (매 실행마다 다름)
        b_bi(t)     : Bias Instability, Gauss-Markov로 천천히 변함
        b_rrw(t)    : Rate Random Walk, 바이어스가 적분되며 드리프트
        n_arw(t)    : Angle Random Walk, 매 스텝 독립 white noise
        K_g · a     : g-sensitivity, 가속도가 자이로 출력을 오염

단위 규칙:
    입력  ω_true : [rad/s]
    출력  ω_meas : [rad/s]
    내부 연산 전부 SI
"""
#*************************************************************************/
# File Name: ./src/core/model/gyroscope.py
# Author: Beomjun Chung
# Updated: 2026-05-17
#
# Description:
#   IMU 센서 모델. trajectory에서 받은 진짜 상태값을 세 센서 모델에 넣어
#   노이즈가 섞인 측정값을 만들어냄. runner.py가 매 스텝마다 호출하는 핵심 인터페이스.
#*************************************************************************/

from __future__ import annotations

import numpy as np
from ..noise.white import ARWNoise
from ..noise.colored import GaussMarkovProcess, BrownNoise
from ..noise.profile import GyroNoiseProfile


class Gyroscope:
    """
    자이로스코프 물리 센서 모델.

    노이즈 성분 4종 + 결정론적 오차 2종을 합산하여
    실제 자이로가 출력하는 측정값을 시뮬레이션.
    """

    def __init__(self, profile: GyroNoiseProfile, seed: int | None = None):
        """
        Args:
            profile: 노이즈 파라미터 묶음 (profile.py 참조)
            seed:    재현성을 위한 난수 시드
        """
        self.profile = profile

        # 난수 시드를 각 생성기에 분산 (같은 시드 주면 상관됨)
        rng = np.random.default_rng(seed)
        seeds = rng.integers(0, 2**31, size=3)

        # ── 노이즈 생성기 ──────────────────────────────────────────
        self._arw = ARWNoise(
            sigma_per_sqrt_s=profile.arw_rad_sqrts,
            size=3,
            seed=int(seeds[0]),
        )
        self._bi = GaussMarkovProcess(
            sigma_bi=profile.bi_sigma_rad_s,
            corr_time_s=profile.bi_corr_time_s,
            size=3,
            seed=int(seeds[1]),
        )
        self._rrw = BrownNoise(
            sigma_rrw=profile.rrw_rad_s_sqrts,
            size=3,
            seed=int(seeds[2]),
        )

        # ── 스케일 팩터 행렬 ───────────────────────────────────────
        # 대각 성분만 사용 (축간 misalignment 무시, 필요시 확장)
        sf = profile.scale_factor_ppm * 1e-6
        self._scale = np.eye(3) + np.diag([sf, sf, sf])

        # ── 상태 초기화 ────────────────────────────────────────────
        # turn-on bias는 매 reset()마다 가우시안으로 랜덤 생성
        # profile의 값을 표준편차로 사용
        self._rng = np.random.default_rng(seed)
        self._turn_on_bias = self._sample_turn_on_bias()

    # ── public API ────────────────────────────────────────────────────────────

    def measure(self, omega_true: np.ndarray, accel_body: np.ndarray,
                dt: float) -> np.ndarray:
        """
        진짜 각속도와 현재 가속도를 받아 측정값 반환.

        Args:
            omega_true:  진짜 각속도 [rad/s], shape (3,)
            accel_body:  body frame 가속도 [m/s²], shape (3,)
                         g-sensitivity 계산에 사용
            dt:          샘플 간격 [s]

        Returns:
            omega_meas: 노이즈가 섞인 자이로 출력 [rad/s], shape (3,)
        """
        # 1. 스케일 팩터 적용
        omega = self._scale @ omega_true

        # 2. 결정론적 바이어스 (turn-on)
        omega = omega + self._turn_on_bias

        # 3. Bias Instability (Gauss-Markov, 천천히 변함)
        omega = omega + self._bi.step(dt)

        # 4. Rate Random Walk (적분 드리프트)
        omega = omega + self._rrw.step(dt)

        # 5. ARW (매 스텝 독립 white noise)
        omega = omega + self._arw.sample(dt)

        # 6. g-sensitivity (가속도 → 자이로 오염)
        omega = omega + self.profile.g_sensitivity_rad_s_per_ms2 @ accel_body

        return omega

    def reset(self) -> None:
        """시뮬 재시작 시 상태 초기화. turn-on bias는 새로 샘플링."""
        self._bi.reset()
        self._rrw.reset()
        self._turn_on_bias = self._sample_turn_on_bias()

    # ── private ───────────────────────────────────────────────────────────────

    def _sample_turn_on_bias(self) -> np.ndarray:
        """
        전원 인가 시 바이어스를 가우시안으로 샘플링.
        profile의 turn_on_bias_rad_s를 표준편차로 사용.
        """
        std = np.abs(self.profile.turn_on_bias_rad_s)
        # std가 0이면 deterministic bias (고정값)
        if np.all(std == 0):
            return self.profile.turn_on_bias_rad_s.copy()
        return self._rng.normal(loc=self.profile.turn_on_bias_rad_s, scale=std * 0.1)

    # ── class method ──────────────────────────────────────────────────────────

    @classmethod
    def from_config(cls, cfg: dict, seed: int | None = None) -> "Gyroscope":
        """config.yaml의 imu.gyroscope 섹션에서 직접 생성."""
        profile = GyroNoiseProfile.from_config(cfg)
        return cls(profile=profile, seed=seed)
