"""
File Name: ./src/ahrs/core/trajectory/base.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  TrajectoryBase ABC — 모든 trajectory의 공통 인터페이스

    purpose:
        trajectory 구현체가 반드시 제공해야 하는 ground truth 데이터를 정의.
        runner.py가 이 인터페이스만 보고 어떤 trajectory든 구동 가능.

    Notes:
        출력 단위: q [x,y,z,w], omega [rad/s], pos/vel [m, m/s], accel_world [m/s²]
        generate()는 dt 간격의 시계열을 한 번에 반환.
        모든 trajectory는 t=0에서 단위 쿼터니언(no rotation)으로 시작하거나
        initial_q를 명시적으로 지정할 수 있음.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class TruthData:
    """ground truth 시계열 데이터 묶음."""
    t:           np.ndarray   # [s],       shape (N,)
    q:           np.ndarray   # [x,y,z,w], shape (N, 4)
    omega:       np.ndarray   # [rad/s],   shape (N, 3)  body frame
    pos:         np.ndarray   # [m],       shape (N, 3)  world frame
    vel:         np.ndarray   # [m/s],     shape (N, 3)  world frame
    accel_world: np.ndarray   # [m/s²],    shape (N, 3)  world frame (gravity 제외)


class TrajectoryBase(ABC):
    """모든 trajectory 클래스의 추상 기반."""

    @abstractmethod
    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        """
        ground truth 시계열 생성.

        Args:
            dt:        샘플 간격 [s]
            duration:  총 시간 [s]
            initial_q: 초기 자세 [x,y,z,w]. None이면 identity.

        Returns:
            TruthData
        """
        ...

    @staticmethod
    def _identity_q() -> np.ndarray:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    @staticmethod
    def _integrate_q(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
        """1차 Euler 쿼터니언 적분. [x,y,z,w] scalar-last."""
        from ahrs.core.orientation.transforms import Transforms
        Omega = np.array([omega[0], omega[1], omega[2], 0.0])
        q_dot = 0.5 * Transforms.quat_mult(q, Omega)
        q_new = q + q_dot * dt
        return Transforms.quat_normalize(q_new)
