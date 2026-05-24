"""
File Name: ./src/ahrs/core/filters/mahony.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Mahony Filter — PI 제어기 기반 AHRS 필터

    purpose:
        오차 벡터(cross product)를 PI 제어기로 피드백하여 자세 보정.
        ki를 통해 자이로 바이어스를 적분하여 추정.

    Notes:
        NED 프레임 중력 컨벤션:
            가속도계는 특정 힘(specific force) = R^T·[0,0,-g] 출력.
            정지·수평 시: accel_body ≈ [0, 0, -g].
            기준벡터 g_ref = [0, 0, -1] (위 방향, specific force 방향).

        cross product 오차:
            e = cross(a_meas, g_pred)  (측정 × 예측)
            e가 양이면 예측을 측정 방향으로 돌려야 함.
"""

from __future__ import annotations

import numpy as np

from ahrs.core.estimator.base import AhrsFilter
from ahrs.utils.transforms import Transforms

class MahonyFilter(AhrsFilter):
    name = "mahony"

    def __init__(self, kp: float = 1.8, ki: float = 0.08, use_magnetometer: bool = True, **kwargs):
        self._kp      = float(kp)
        self._ki      = float(ki)
        self._use_mag = bool(use_magnetometer)
        self._q       = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias    = np.zeros(3, dtype=float)

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # --- accel error ---
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            a_hat = accel / a_norm
            # NED specific force reference: [0,0,-1] (up direction)
            # 예측: R^T · [0,0,-1] = expected accel body direction
            g_ref  = np.array([0.0, 0.0, -1.0])
            g_pred = Transforms.world2body(self._q, g_ref)
            # e = cross(measured, predicted)
            e_acc = np.cross(a_hat, g_pred)
        else:
            e_acc = np.zeros(3)

        # --- mag error ---
        if self._use_mag:
            m_norm = np.linalg.norm(mag)
            if m_norm > 1e-6:
                # normalizaed mag values
                m_hat = mag / m_norm
                # 측정 자기장을 world frame으로 투영, 수평 성분만 (heading 전용)
                m_world = Transforms.body2world(self._q, m_hat)
                # horizontal magnitude
                h_horiz = np.sqrt(m_world[0]**2 + m_world[1]**2)
                m_ref_world = np.array([h_horiz, 0.0, m_world[2]])
                m_ref_world_norm = np.linalg.norm(m_ref_world)
                if m_ref_world_norm > 1e-6:
                    m_ref_world = m_ref_world / m_ref_world_norm
                    m_ref_body = Transforms.world2body(self._q, m_ref_world)
                    e_mag = np.cross(m_hat, m_ref_body)
                else:
                    e_mag = np.zeros(3)
            else:
                e_mag = np.zeros(3)
        else:
            e_mag = np.zeros(3)

        # --- pi controller ---
        e = e_acc + e_mag
        self._bias -= self._ki * e * dt
        omega_corr = gyro - self._bias + self._kp * e

        # --- gyro integral ---
        Omega = np.array([omega_corr[0], omega_corr[1], omega_corr[2], 0.0])
        q_dot = 0.5 * Transforms.quat_mult(self._q, Omega)
        self._q = Transforms.quat_normalize(self._q + q_dot * dt)

        return self._q.copy(), self._bias.copy()

    def reset(self) -> None:
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)