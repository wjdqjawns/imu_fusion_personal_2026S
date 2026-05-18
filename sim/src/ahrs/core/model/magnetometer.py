"""
magnetometer.py — 자기계 센서 모델

운동학 모델 (Kinematic Model):
    지구 자기장 기준벡터를 body frame으로 변환한 뒤 왜곡을 더함.

    m_true  = R^T · m_world       (body frame의 진짜 자기장)
    m_meas  = W · (m_true + h) + n + d(t)

    각 항의 의미:
        h       : hard iron [uT]
                  주변 강자성체(배터리, 모터 등)에 의한 고정 오프셋
                  센서를 어느 방향으로 돌려도 일정하게 더해짐
                  → 구형 분포가 원점에서 벗어남 (중심 이동)

        W       : soft iron 행렬 [dimensionless, 3×3]
                  주변 연자성체(철 구조물)에 의한 방향별 감도 왜곡
                  이상적 센서면 단위행렬, 왜곡 있으면 비대각 성분 생김
                  → 구형 분포가 타원체로 찌그러짐

        n       : 화이트 노이즈 [uT]
        d(t)    : 시간 변하는 외부 자기 외란 [uT]
                  (모터 전류 변화, 근처 금속 이동 등)

왜 구형 → 타원체인가:
    보정이 완벽하면 어느 방향으로 돌려도 자기장 크기 |m|이 일정
    → 3D scatter가 원점 중심 구형
    hard/soft iron 왜곡이 있으면 타원체로 나타남
    캘리브레이션 = 타원체 → 구형으로 역변환하는 W^-1, h 추정

AHRS 맥락:
    자기계는 yaw(heading) 추정의 유일한 외부 기준.
    자기 외란 구간에서 mag update를 억제하지 않으면 yaw 오차 급증.
"""
#*************************************************************************/
# File Name: ./src/core/model/magnetometer.py
# Author: Beomjun Chung
# Updated: 2026-05-17
#
# Description:
#   IMU 센서 모델. trajectory에서 받은 진짜 상태값을 세 센서 모델에 넣어
#   노이즈가 섞인 측정값을 만들어냄. runner.py가 매 스텝마다 호출하는 핵심 인터페이스.
#*************************************************************************/

from __future__ import annotations

import numpy as np

from ..noise.colored import GaussMarkovProcess
from ..noise.profile import MagNoiseProfile


class Magnetometer:
    """자기계 물리 센서 모델."""

    def __init__(self, profile: MagNoiseProfile, seed: int | None = None):
        """
        Args:
            profile: 노이즈 + 왜곡 파라미터 묶음
            seed:    난수 시드
        """
        self.profile = profile
        self._rng = np.random.default_rng(seed)

        # Bias Instability (자기계도 느린 바이어스 변동 있음)
        if profile.bias_instability_uT > 0:
            self._bi = GaussMarkovProcess(
                sigma_bi=profile.bias_instability_uT / 0.6642,
                corr_time_s=profile.bi_corr_time_s,
                size=3,
                seed=int(self._rng.integers(0, 2**31)),
            )
        else:
            self._bi = None

        # 외란 상태
        self._disturbance: np.ndarray = np.zeros(3)
        self._disturbance_active: bool = False

    # ── public API ────────────────────────────────────────────────────────────

    def measure(self, mag_true: np.ndarray, dt: float) -> np.ndarray:
        """
        진짜 자기장(body frame)을 받아 왜곡+노이즈가 섞인 측정값 반환.

        Args:
            mag_true: body frame 진짜 자기장 [uT], shape (3,)
                      = R^T · m_world  (trajectory에서 계산)
            dt:       샘플 간격 [s]

        Returns:
            mag_meas: 측정값 [uT], shape (3,)
        """
        # 1. Hard iron: 고정 오프셋 추가
        m = mag_true + self.profile.hard_iron_uT

        # 2. Soft iron: 방향별 감도 왜곡 (행렬 곱)
        m = self.profile.soft_iron_matrix @ m

        # 3. Bias Instability
        if self._bi is not None:
            m = m + self._bi.step(dt)

        # 4. White noise
        m = m + self._rng.standard_normal(3) * self.profile.noise_std_uT

        # 5. 외부 자기 외란
        if self._disturbance_active:
            m = m + self._disturbance

        return m

    def set_disturbance(self, disturbance_uT: np.ndarray) -> None:
        """외부 자기 외란 설정. runner에서 이벤트 타임에 호출."""
        self._disturbance = np.array(disturbance_uT)
        self._disturbance_active = True

    def clear_disturbance(self) -> None:
        self._disturbance = np.zeros(3)
        self._disturbance_active = False

    def is_disturbed(self, mag_meas: np.ndarray,
                     reference_norm_uT: float,
                     threshold: float = 0.2) -> bool:
        """
        자기 외란 감지.
        기준 자기장 크기와 측정값 크기를 비교.

        Args:
            mag_meas:          측정된 자기장 [uT]
            reference_norm_uT: 기준 자기장 크기 [uT]
            threshold:         허용 비율 (기본 20%)

        Returns:
            True if 자기 외란 감지됨
        """
        measured_norm = np.linalg.norm(mag_meas)
        return abs(measured_norm - reference_norm_uT) > threshold * reference_norm_uT

    def reset(self) -> None:
        if self._bi is not None:
            self._bi.reset()
        self.clear_disturbance()

    @classmethod
    def from_config(cls, cfg: dict, seed: int | None = None) -> "Magnetometer":
        profile = MagNoiseProfile.from_config(cfg)
        return cls(profile=profile, seed=seed)
