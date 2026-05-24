"""
File Name: ./src/ahrs/core/model/magnetometer.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    자기계 물리 센서 모델.

    측정 방정식:
        m_meas = W · (m_true + h) + b_bi(t) + n(t)

        h   : hard iron [µT]  — 주변 강자성체 고정 오프셋 → 구 중심 이동
        W   : soft iron [3×3] — 방향별 감도 왜곡 → 구 → 타원체
        b_bi: Bias Instability (Gauss-Markov)
        n   : white noise

    외란(자기 간섭) 처리:
        runner가 mag_meas에 직접 더하는 방식으로 처리.
        센서 모델은 환경 외란을 모름 — 센서 내부 노이즈만 담당.

    캘리브레이션 맥락:
        보정 전 scatter = 타원체 (hard/soft iron 포함)
        보정 후 scatter ≈ 구형 (중심 원점, 균일 반지름)
"""

from __future__ import annotations

import numpy as np

from ahrs.core.noise.colored import GaussMarkovProcess
from ahrs.core.noise.profile import MagNoiseProfile
from ahrs.utils.logger import get_logger

logger = get_logger(__name__)

class Magnetometer:
    def __init__(self, profile: MagNoiseProfile, seed: int | None = None):
        self.profile = profile
        self._rng    = np.random.default_rng(seed)

        if profile.bias_instability_uT > 0:
            self._bi = GaussMarkovProcess(
                sigma_bi     = profile.bias_instability_uT / 0.6642,
                corr_time_s  = profile.bi_corr_time_s,
                size         = 3,
                seed         = int(self._rng.integers(0, 2**31)),
            )
        else:
            self._bi = None

    # ── public API ────────────────────────────────────────────────────────────

    def measure(self, mag_true: np.ndarray, dt: float) -> np.ndarray:
        """
        body frame 진짜 자기장 → 왜곡 + 노이즈 측정값.

        Args:
            mag_true: R^T · m_world [µT], shape (3,)
            dt:       sample interval [s]
        """
        # 1. Hard iron: 고정 오프셋
        m = mag_true + self.profile.hard_iron_uT

        # 2. Soft iron: 방향별 왜곡
        m = self.profile.soft_iron_matrix @ m

        # 3. Bias Instability (Gauss-Markov)
        if self._bi is not None:
            m = m + self._bi.step(dt)

        # 4. White noise
        m = m + self._rng.standard_normal(3) * self.profile.noise_std_uT

        return m

    def reset(self) -> None:
        if self._bi is not None:
            self._bi.reset()

    # ── utility ───────────────────────────────────────────────────────────────

    @staticmethod
    def is_disturbed(
        mag_meas: np.ndarray,
        reference_norm_uT: float,
        threshold: float = 0.2,
    ) -> bool:
        """
        자기 외란 감지 유틸리티.
        기준 자기장 크기 대비 측정값 크기 비교.

        필터 내부 또는 분석 코드에서 직접 호출용.
        (IMUMeasurement에 포함되지 않음 — 필터가 직접 판단)
        """
        return abs(np.linalg.norm(mag_meas) - reference_norm_uT) > threshold * reference_norm_uT

    @classmethod
    def from_config(cls, cfg: dict, seed: int | None = None) -> "Magnetometer":
        return cls(profile=MagNoiseProfile.from_config(cfg), seed=seed)