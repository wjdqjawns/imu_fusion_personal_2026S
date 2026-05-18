"""
File Name: ./src/ahrs/core/trajectory/figure8.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  8자 궤도 trajectory — roll/pitch 격렬 변화

    purpose:
        roll과 pitch가 크게 변하는 기동. 필터 수렴 성능과 가속도 외란 응답 평가.

    Notes:
        - max_tilt_deg: 최대 기울기 (roll/pitch 진폭)
        - period_s: 8자 한 주기
        - roll은 sin, pitch는 cos 조합으로 부드러운 8자 생성
        - omega는 자세 미분으로 계산
"""

from __future__ import annotations

import numpy as np

from ahrs.core.trajectory.base import TrajectoryBase, TruthData
from ahrs.core.orientation.transforms import Transforms


class Figure8Trajectory(TrajectoryBase):
    """8자 궤도 trajectory."""

    def __init__(self, max_tilt_deg: float = 45.0, period_s: float = 8.0):
        self._tilt   = np.deg2rad(max_tilt_deg)
        self._period = float(period_s)

    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        t = np.arange(0.0, duration, dt)
        n = len(t)

        q_arr     = np.zeros((n, 4), dtype=float)
        omega_arr = np.zeros((n, 3), dtype=float)

        w = 2.0 * np.pi / self._period

        # 자세: roll = A·sin(wt), pitch = A·sin(2wt)/2, yaw=0 (heading 고정)
        roll_series  = self._tilt * np.sin(w * t)
        pitch_series = self._tilt * 0.5 * np.sin(2 * w * t)
        yaw_series   = np.zeros(n)

        # 자세 미분 → omega (body frame)
        roll_dot  = self._tilt * w * np.cos(w * t)
        pitch_dot = self._tilt * w * np.cos(2 * w * t)
        yaw_dot   = np.zeros(n)

        for k in range(n):
            q = Transforms.euler_to_quat(roll_series[k], pitch_series[k], yaw_series[k])
            if k == 0 and initial_q is not None:
                q = np.asarray(initial_q, dtype=float)

            # euler dot → body omega (ZYX Jacobian 역변환)
            # [p, q, r] = J^-1 · [roll_dot, pitch_dot, yaw_dot]
            r, p, y = roll_series[k], pitch_series[k], yaw_series[k]
            omega_body = self._euler_dot_to_body_omega(
                roll_dot[k], pitch_dot[k], yaw_dot[k], r, p
            )

            q_arr[k]     = q
            omega_arr[k] = omega_body

        return TruthData(
            t=t, q=q_arr, omega=omega_arr,
            pos=np.zeros((n, 3)), vel=np.zeros((n, 3)), accel_world=np.zeros((n, 3))
        )

    @staticmethod
    def _euler_dot_to_body_omega(roll_dot: float, pitch_dot: float, yaw_dot: float,
                                  roll: float, pitch: float) -> np.ndarray:
        """
        ZYX Euler rate → body frame angular velocity.
        [p, q, r]^T = B · [roll_dot, pitch_dot, yaw_dot]^T
        B 행렬 (ZYX):
            p = roll_dot - yaw_dot·sin(pitch)
            q = pitch_dot·cos(roll) + yaw_dot·sin(roll)·cos(pitch)
            r = -pitch_dot·sin(roll) + yaw_dot·cos(roll)·cos(pitch)
        """
        sr, cr = np.sin(roll), np.cos(roll)
        sp, cp = np.sin(pitch), np.cos(pitch)

        p = roll_dot - yaw_dot * sp
        q = pitch_dot * cr + yaw_dot * sr * cp
        r = -pitch_dot * sr + yaw_dot * cr * cp
        return np.array([p, q, r], dtype=float)

    @classmethod
    def from_config(cls, cfg: dict) -> "Figure8Trajectory":
        return cls(
            max_tilt_deg=float(cfg.get("max_tilt_deg", 45.0)),
            period_s=float(cfg.get("period_s", 8.0)),
        )
