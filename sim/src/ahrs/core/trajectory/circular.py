"""
File Name: ./src/ahrs/core/trajectory/circular.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  원형 선회 trajectory — yaw rate 일정, 선택적 roll/pitch 유지

    purpose:
        일정 yaw rate로 수평 또는 경사 선회 시뮬레이션.
        complementary/Mahony/Madgwick/EKF 필터의 yaw 추정 성능 검증.

    Notes:
        - angular_velocity_deg_s: yaw rate [deg/s]
        - roll_deg: 선회 중 일정 roll 각도 (경사 선회, 0이면 수평)
        - pitch_deg: 선회 중 일정 pitch 각도
        - body frame omega는 일정 자세에서의 yaw rate를 body z축 성분으로 변환
"""

from __future__ import annotations

import numpy as np

from ahrs.core.trajectory.base import TrajectoryBase, TruthData
from ahrs.core.orientation.transforms import Transforms


class CircularTrajectory(TrajectoryBase):
    """일정 yaw rate 원형 선회 trajectory."""

    def __init__(self, angular_velocity_deg_s: float = 20.0,
                 roll_deg: float = 0.0, pitch_deg: float = 0.0):
        self._yaw_rate = np.deg2rad(angular_velocity_deg_s)  # [rad/s]
        self._roll     = np.deg2rad(roll_deg)
        self._pitch    = np.deg2rad(pitch_deg)

    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        t = np.arange(0.0, duration, dt)
        n = len(t)

        q_arr       = np.zeros((n, 4), dtype=float)
        omega_arr   = np.zeros((n, 3), dtype=float)
        pos_arr     = np.zeros((n, 3), dtype=float)
        vel_arr     = np.zeros((n, 3), dtype=float)
        accel_arr   = np.zeros((n, 3), dtype=float)

        # 초기 자세: 지정된 roll/pitch, yaw=0
        q = Transforms.euler_to_quat(self._roll, self._pitch, 0.0)
        if initial_q is not None:
            q = np.asarray(initial_q, dtype=float)

        # body frame omega: 자세가 (roll, pitch)로 고정된 상태에서 world yaw rate를
        # body z축으로 투영. 수평 선회(roll=pitch=0)면 omega_body = [0, 0, yaw_rate].
        R0 = Transforms.quat_to_dcm(q)
        # world z축 yaw rate → body frame
        omega_world_yaw = np.array([0.0, 0.0, self._yaw_rate])
        omega_body = R0.T @ omega_world_yaw  # constant throughout circular flight

        for k in range(n):
            q_arr[k]     = q
            omega_arr[k] = omega_body
            q = self._integrate_q(q, omega_body, dt)

        return TruthData(t=t, q=q_arr, omega=omega_arr,
                         pos=pos_arr, vel=vel_arr, accel_world=accel_arr)

    @classmethod
    def from_config(cls, cfg: dict) -> "CircularTrajectory":
        return cls(
            angular_velocity_deg_s=float(cfg.get("angular_velocity_deg_s", 20.0)),
            roll_deg=float(cfg.get("roll_deg", 0.0)),
            pitch_deg=float(cfg.get("pitch_deg", 0.0)),
        )
