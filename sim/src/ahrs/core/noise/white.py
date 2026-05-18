"""
File Name: ./src/ahrs/core/noise/white.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  White noise 생성기 — ARWNoise (자이로), VRWNoise (가속도계)

    purpose:
        Allan variance 기울기 -1/2 구간에 해당하는 white noise.
        매 샘플 독립 가우시안 노이즈를 출력함.

    Notes:
        - ARW [deg/√h] → rad/√s 단위변환은 profile.py에서 처리
        - 내부 단위: sigma_per_sqrt_s [rad/√s 또는 m/s²/√s]
        - sigma_discrete = sigma_per_sqrt_s / sqrt(dt)
"""

from __future__ import annotations

import numpy as np


class ARWNoise:
    """
    Angle Random Walk — 자이로 white noise.
    Allan deviation 기울기 -1/2 구간.
    """

    def __init__(self, sigma_per_sqrt_s: float, size: int = 3,
                 seed: int | None = None):
        """
        Args:
            sigma_per_sqrt_s: 연속 PSD의 제곱근 [rad/√s]
            size:             출력 벡터 크기
            seed:             난수 시드
        """
        self._sigma = float(sigma_per_sqrt_s)
        self._size  = size
        self._rng   = np.random.default_rng(seed)

    def sample(self, dt: float) -> np.ndarray:
        """
        한 샘플 반환.
        이산화: σ_discrete = σ_continuous / √dt
        """
        sigma_d = self._sigma / np.sqrt(dt)
        return self._rng.normal(0.0, sigma_d, self._size)


class VRWNoise:
    """
    Velocity Random Walk — 가속도계 white noise.
    Allan deviation 기울기 -1/2 구간.
    """

    def __init__(self, sigma_per_sqrt_s: float, size: int = 3,
                 seed: int | None = None):
        """
        Args:
            sigma_per_sqrt_s: 연속 PSD의 제곱근 [m/s²/√s]
        """
        self._sigma = float(sigma_per_sqrt_s)
        self._size  = size
        self._rng   = np.random.default_rng(seed)

    def sample(self, dt: float) -> np.ndarray:
        sigma_d = self._sigma / np.sqrt(dt)
        return self._rng.normal(0.0, sigma_d, self._size)
