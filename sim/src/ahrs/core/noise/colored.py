"""
File Name: ./src/ahrs/core/noise/colored.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Colored noise 생성기 — GaussMarkovProcess (BI), BrownNoise (RRW)

    purpose:
        Allan variance의 플랫 구간(Bias Instability)과 기울기 +1/2 구간(RRW)을
        시뮬레이션하는 시간 상관 노이즈 생성기.

    Notes:
        GaussMarkovProcess:
            dx = -(1/τ)·x·dt + σ_d·dW
            이산화: x[k+1] = exp(-dt/τ)·x[k] + σ_d·√(1-exp(-2dt/τ))·n
            Allan curve 최솟값 = σ_bi = σ_d · √(τ/2) → σ_d 계산식 포함

        BrownNoise (Rate Random Walk):
            적분 랜덤워크. 자이로 바이어스가 시간이 지남에 따라 드리프트.
            x[k+1] = x[k] + σ_rrw·√dt·n
"""

from __future__ import annotations

import numpy as np


class GaussMarkovProcess:
    """
    1차 Gauss-Markov 프로세스 — Bias Instability 모델링.

    Allan deviation 최솟값(BI)으로 파라미터 초기화.
    σ_drive = σ_bi * √(2 / τ)  (Allan BI 정의에서 유도)
    """

    def __init__(self, sigma_bi: float, corr_time_s: float,
                 size: int = 3, seed: int | None = None):
        """
        Args:
            sigma_bi:      Allan deviation 최솟값 [rad/s or m/s² or uT]
                           프로세스 정상상태 표준편차
            corr_time_s:   상관시간 τ [s]
            size:          상태벡터 크기
            seed:          난수 시드
        """
        self._tau    = float(corr_time_s)
        self._size   = size
        self._rng    = np.random.default_rng(seed)

        # σ_drive: 이산 스텝 당 구동 노이즈 (dt 없이 저장, step()에서 사용)
        # 정상상태 분산 = σ_drive² * τ / 2 = σ_bi²  → σ_drive = σ_bi * √(2/τ)
        self._sigma_drive = float(sigma_bi) * np.sqrt(2.0 / self._tau)

        self._state = np.zeros(size, dtype=float)

    def step(self, dt: float) -> np.ndarray:
        """
        한 스텝 진행, 현재 상태 반환.

        이산화:
            decay    = exp(-dt/τ)
            σ_d      = σ_drive · √((1 - decay²)/2 · τ)
                      ≈ σ_drive · √dt  (dt << τ 근사)
        """
        decay = np.exp(-dt / self._tau)
        # 정확한 이산화 분산: Var = σ_drive² * τ/2 * (1 - decay²)
        sigma_d = self._sigma_drive * np.sqrt(self._tau / 2.0 * (1.0 - decay**2))
        self._state = decay * self._state + self._rng.normal(0.0, sigma_d, self._size)
        return self._state.copy()

    def reset(self) -> None:
        self._state = np.zeros(self._size, dtype=float)


class BrownNoise:
    """
    Brown noise (적분 랜덤워크) — Rate Random Walk 모델링.

    Allan deviation 기울기 +1/2 구간.
    x[k+1] = x[k] + σ_rrw · √dt · n
    """

    def __init__(self, sigma_rrw: float, size: int = 3,
                 seed: int | None = None):
        """
        Args:
            sigma_rrw: RRW 계수 [rad/s/√s or m/s²/√s]
                       Allan deviation +1/2 기울기 구간에서 읽은 값
        """
        self._sigma = float(sigma_rrw)
        self._size  = size
        self._rng   = np.random.default_rng(seed)
        self._state = np.zeros(size, dtype=float)

    def step(self, dt: float) -> np.ndarray:
        """한 스텝 진행, 누적 상태 반환."""
        self._state += self._rng.normal(0.0, self._sigma * np.sqrt(dt), self._size)
        return self._state.copy()

    def reset(self) -> None:
        self._state = np.zeros(self._size, dtype=float)
