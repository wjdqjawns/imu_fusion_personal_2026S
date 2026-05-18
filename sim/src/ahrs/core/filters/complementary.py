"""
File Name: ./src/ahrs/core/filters/complementary.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Complementary Filter — 고주파 자이로 + 저주파 accel/mag 혼합

    purpose:
        alpha를 주파수 경계로 두 신호를 합성.
        alpha ≈ 0.98 → cutoff ≈ (1-α)/(2π·α·dt) ≈ 0.08 Hz
        단순, 빠르고 구현이 명확한 기본 필터.

    Notes:
        - accel: gravity 기준벡터 추정, NED에서 [0,0,g]
        - mag: yaw 기준벡터. use_magnetometer=False면 yaw는 자이로만으로 적분
        - 선형 가속도 외란 시 accel update 약화 없음 → 단점
"""

from __future__ import annotations

import numpy as np

from ahrs.core.filters.base import AhrsFilter
from ahrs.core.orientation.transforms import Transforms

class ComplementaryFilter(AhrsFilter):
    """Complementary Filter. alpha 가중 혼합."""

    name = "complementary"

    def __init__(self, alpha: float = 0.98,
                 use_magnetometer: bool = True, **kwargs):
        self._alpha   = float(alpha)
        self._use_mag = bool(use_magnetometer)
        self._q       = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # identity
        self._bias    = np.zeros(3, dtype=float)

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # 1. 자이로 적분 (high-pass)
        omega = gyro - self._bias
        Omega = np.array([omega[0], omega[1], omega[2], 0.0])
        q_dot = 0.5 * Transforms.quat_mult(self._q, Omega)
        q_gyro = Transforms.quat_normalize(self._q + q_dot * dt)

        # 2. accel 기반 기울기 추정 (tilt only: roll, pitch)
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            a_hat = accel / a_norm
            # 현재 추정 자세로 중력 방향 예측 (NED에서 g=[0,0,1] 정규화)
            g_ref = np.array([0.0, 0.0, 1.0])
            # 오차 벡터: cross product (예측 방향 × 측정 방향)
            g_pred = self._rotate_inv(q_gyro, g_ref)
            e_acc = np.cross(g_pred, a_hat)
        else:
            e_acc = np.zeros(3)

        # 3. mag 기반 heading 보정
        if self._use_mag:
            m_norm = np.linalg.norm(mag)
            if m_norm > 1e-6:
                m_hat = mag / m_norm
                # 현재 자세로 예측한 자기 기준벡터를 body frame으로
                # 수평 성분만 사용 (tilt compensated magnetometer)
                m_body_pred = self._tilt_compensated_mag_ref(q_gyro)
                e_mag = np.cross(m_body_pred, m_hat)
            else:
                e_mag = np.zeros(3)
        else:
            e_mag = np.zeros(3)

        # 4. 보정 자세 (low-pass)
        # accel+mag로 자세를 직접 추정하고 alpha로 혼합
        q_corr = self._accel_mag_to_quat(accel, mag, q_gyro)
        q_new = Transforms.quat_normalize(
            self._slerp_q(q_gyro, q_corr, 1.0 - self._alpha)
        )
        self._q = q_new
        return self._q.copy(), self._bias.copy()

    def reset(self) -> None:
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)

    # helper
    @staticmethod
    def _rotate_inv(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """world → body: v_body = q* ⊗ [v,0] ⊗ q"""
        qv = np.array([v[0], v[1], v[2], 0.0])
        r = Transforms.quat_mult(
            Transforms.quat_mult(Transforms.quat_conjugate(q), qv), q
        )
        return r[:3]

    def _tilt_compensated_mag_ref(self, q: np.ndarray) -> np.ndarray:
        """
        자기 기준벡터의 body frame 예측.
        자기장 기준벡터가 없으므로 수평 성분만 추출하는 대신
        측정 자기장 자체를 참조로 사용할 수 없어 단위벡터 north 방향 근사.
        """
        # body frame에서 north 방향 ≈ DCM 1행 (world x→body)
        R = Transforms.quat_to_dcm(q)
        return R[0]  # north 방향 body projection

    @staticmethod
    def _accel_mag_to_quat(accel: np.ndarray, mag: np.ndarray,
                            q_ref: np.ndarray) -> np.ndarray:
        """
        accel+mag로 자세 쿼터니언 추정 (tilt+heading).
        roll/pitch는 accel에서, yaw는 tilt-compensated mag에서.
        """
        a_norm = np.linalg.norm(accel)
        if a_norm < 1e-6:
            return q_ref

        a = accel / a_norm

        # roll, pitch from accel (NED: g=[0,0,+1] normalized)
        roll  = np.arctan2(-a[1], -a[2])
        pitch = np.arctan2(a[0], np.sqrt(a[1]**2 + a[2]**2))

        # yaw from tilt-compensated mag
        m_norm = np.linalg.norm(mag)
        if m_norm < 1e-6:
            yaw = Transforms.quat_to_euler(q_ref)[2]
        else:
            m = mag / m_norm
            sr, cr = np.sin(roll), np.cos(roll)
            sp, cp = np.sin(pitch), np.cos(pitch)
            # tilt compensate
            mx = m[0]*cp + m[1]*sp*sr + m[2]*sp*cr
            my = m[1]*cr - m[2]*sr
            yaw = np.arctan2(-my, mx)

        return Transforms.euler_to_quat(roll, pitch, yaw)

    @staticmethod
    def _slerp_q(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """SLERP 보간. t=0→q0, t=1→q1."""
        dot = np.dot(q0, q1)
        if dot < 0:
            q1 = -q1
            dot = -dot
        dot = np.clip(dot, -1.0, 1.0)
        if dot > 0.9995:
            return Transforms.quat_normalize(q0 + t * (q1 - q0))
        theta0 = np.arccos(dot)
        theta  = theta0 * t
        sin_t0 = np.sin(theta0)
        s0 = np.cos(theta) - dot * np.sin(theta) / sin_t0
        s1 = np.sin(theta) / sin_t0
        return Transforms.quat_normalize(s0 * q0 + s1 * q1)