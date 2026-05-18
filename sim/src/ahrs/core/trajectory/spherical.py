"""
File Name: ./src/ahrs/core/trajectory/spherical.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  구형 회전 trajectory — 자기계 캘리브레이션용

    purpose:
        모든 방향을 고르게 커버하는 회전 경로. 자기계 타원체 피팅(ellipsoid fit)에
        필요한 충분한 방향 다양성을 보장.

    Notes:
        - 나선형(spiral) 경로로 구면 전체를 커버
        - rate_deg_s: 평균 회전 속도
        - 3축 sin/cos 조합으로 구형 경로 근사
"""

from __future__ import annotations

import numpy as np

from ahrs.core.trajectory.base import TrajectoryBase, TruthData
from ahrs.core.orientation.transforms import Transforms


class SphericalTrajectory(TrajectoryBase):
    """구형 회전 trajectory — 자기계 캘리브레이션용."""

    def __init__(self, rate_deg_s: float = 30.0):
        self._rate = np.deg2rad(rate_deg_s)  # [rad/s]

    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        t = np.arange(0.0, duration, dt)
        n = len(t)

        q_arr     = np.zeros((n, 4), dtype=float)
        omega_arr = np.zeros((n, 3), dtype=float)

        # 세 축에 다른 주파수로 sinusoidal 회전 → 구면 방향 커버
        # 황금비(1.618) 주파수 비율로 리사주 패턴 최소화
        # π 인수: 각도 진폭 = ±π (±180°) 보장 → 전체 구면 커버
        f1 = self._rate
        f2 = self._rate * 1.618
        f3 = self._rate * 2.618

        # 시간에 따라 변하는 omega_body
        # integral = π·sin(fi·t) → 진폭 ±π rad (±180°)
        omega_x = np.pi * f1 * np.cos(f1 * t)
        omega_y = np.pi * f2 * np.cos(f2 * t + np.pi / 3)
        omega_z = np.pi * f3 * np.cos(f3 * t + 2 * np.pi / 3)

        q = initial_q if initial_q is not None else self._identity_q()
        q = np.asarray(q, dtype=float)

        for k in range(n):
            omega_body = np.array([omega_x[k], omega_y[k], omega_z[k]])
            q_arr[k]     = q
            omega_arr[k] = omega_body
            q = self._integrate_q(q, omega_body, dt)

        return TruthData(
            t=t, q=q_arr, omega=omega_arr,
            pos=np.zeros((n, 3)), vel=np.zeros((n, 3)), accel_world=np.zeros((n, 3))
        )

    @classmethod
    def from_config(cls, cfg: dict) -> "SphericalTrajectory":
        return cls(rate_deg_s=float(cfg.get("rate_deg_s", 30.0)))
