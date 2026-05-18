"""
File Name: ./src/ahrs/core/trajectory/static.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  정지 trajectory — Allan variance 분석용

    purpose:
        센서가 완전 정지 상태. omega=0, 자세 변화 없음.
        Allan variance 측정 시 정적 노이즈 특성만 분리하여 분석.

    Notes:
        duration 300s 이상 권장 (Allan variance tau_max = duration/5)
"""

from __future__ import annotations

import numpy as np

from ahrs.core.trajectory.base import TrajectoryBase, TruthData


class StaticTrajectory(TrajectoryBase):
    """완전 정지 trajectory."""

    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        t = np.arange(0.0, duration, dt)
        n = len(t)

        q0 = initial_q if initial_q is not None else self._identity_q()
        q0 = np.asarray(q0, dtype=float)

        q           = np.tile(q0, (n, 1))
        omega       = np.zeros((n, 3), dtype=float)
        pos         = np.zeros((n, 3), dtype=float)
        vel         = np.zeros((n, 3), dtype=float)
        accel_world = np.zeros((n, 3), dtype=float)

        return TruthData(t=t, q=q, omega=omega,
                         pos=pos, vel=vel, accel_world=accel_world)

    @classmethod
    def from_config(cls, cfg: dict) -> "StaticTrajectory":
        return cls()
