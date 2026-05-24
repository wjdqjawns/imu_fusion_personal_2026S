"""
File Name: ./src/ahrs/core/filters/ekf.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
    EKF (Extended Kalman Filter)

    purpose:
        상태: q(4) + gyro_bias(3) = dim 7
        predict: 자이로 적분 + 공분산 전파
        update: accel(gravity), mag(heading) 각각 분리 수행

    Notes:
        - 공분산: error-state (quaternion 섭동 3차원 + bias 3 = 6×6)
        - accel update: |a_meas - g| < threshold 조건에서만 수행
        - mag update: |m_meas| ≈ |m_ref| 조건에서만 수행
        - 두 update를 별도 step으로 분리 (수치 안정성)
        - quaternion norm 재정규화 매 스텝 수행
        - mag 기준벡터: runner에서 inject하거나 초기 측정값으로 자동 추정
"""

from __future__ import annotations

import numpy as np

from ahrs.core.estimator.base import AhrsFilter
from ahrs.utils.transforms import Transforms

class EkfFilter(AhrsFilter):
    name = "ekf"

    def __init__(
        self,
        gyro_noise_std_rad_s: float = 0.003,
        gyro_bias_rw_std_rad_s2: float = 0.0005,
        accel_meas_std_m_s2: float = 0.05,
        mag_meas_std_uT: float = 0.1,
        init_attitude_std_deg: float = 10.0,
        init_bias_std_rad_s: float = 0.01,
        accel_threshold_g: float = 0.2,
        gravity_m_s2: float = 9.80665,
        mag_ref_uT: np.ndarray | None = None,
        **kwargs,
    ):
        self._sigma_gyro    = float(gyro_noise_std_rad_s)
        self._sigma_bias_rw = float(gyro_bias_rw_std_rad_s2)
        self._sigma_accel   = float(accel_meas_std_m_s2)
        self._sigma_mag     = float(mag_meas_std_uT)
        self._g             = float(gravity_m_s2)
        self._accel_thr     = float(accel_threshold_g) * self._g

        # state vector: q [x,y,z,w] + bias [3]; 7-dim
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)

        # covariance: error-state 6×6 (attitude error dθ[3] + bias[3])
        init_att_var  = np.deg2rad(init_attitude_std_deg) ** 2
        init_bias_var = float(init_bias_std_rad_s) ** 2
        self._P = np.block([
            [init_att_var  * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)),          init_bias_var * np.eye(3)],
        ])

        # 자기장 기준벡터 (NED, [uT]). None이면 첫 측정으로 초기화.
        self._mag_ref: np.ndarray | None = (
            np.asarray(mag_ref_uT, dtype=float) if mag_ref_uT is not None else None
        )
        self._mag_ref_norm: float = (
            float(np.linalg.norm(mag_ref_uT)) if mag_ref_uT is not None else 0.0
        )

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def set_mag_ref(self, mag_ned_uT: np.ndarray) -> None:
        """자기장 기준벡터 설정. runner에서 환경 설정 후 호출."""
        self._mag_ref      = np.asarray(mag_ned_uT, dtype=float)
        self._mag_ref_norm = float(np.linalg.norm(self._mag_ref))

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # 자기장 기준벡터 초기화 (첫 스텝)
        if self._mag_ref is None:
            R = Transforms.quat_to_dcm(self._q)
            self._mag_ref = R @ mag  # body→world
            self._mag_ref_norm = float(np.linalg.norm(self._mag_ref))

        # 1. Predict
        self._predict(gyro, dt)

        # 2. Accel update (중력 기준벡터)
        if abs(np.linalg.norm(accel) - self._g) < self._accel_thr:
            self._update_accel(accel)

        # 3. Mag update (heading 기준벡터)
        if self._mag_ref_norm > 1e-6:
            mag_norm = np.linalg.norm(mag)
            if abs(mag_norm - self._mag_ref_norm) < 0.2 * self._mag_ref_norm:
                self._update_mag(mag)

        return self._q.copy(), self._bias.copy()

    def reset(self) -> None:
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)
        init_att_var  = np.deg2rad(10.0) ** 2
        self._P = np.block([
            [init_att_var * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)),         0.01**2 * np.eye(3)],
        ])

    # ── private: predict ──────────────────────────────────────────────────────
    def _predict(self, gyro: np.ndarray, dt: float) -> None:
        omega = gyro - self._bias

        # quaternion integral
        Omega = np.array([omega[0], omega[1], omega[2], 0.0])
        q_dot = 0.5 * Transforms.quat_mult(self._q, Omega)
        self._q = Transforms.quat_normalize(self._q + q_dot * dt)

        # error-state covariance propagation
        # F ≈ I + A·dt  where A:
        #   dδθ/dt = -[ω×] δθ - δb
        #   dδb/dt = 0
        omega_skew = self._skew(omega)
        A = np.zeros((6, 6))
        A[:3, :3] = -omega_skew
        A[:3, 3:] = -np.eye(3)
        F = np.eye(6) + A * dt

        # process noise
        Q = np.zeros((6, 6))
        Q[:3, :3] = (self._sigma_gyro ** 2) * dt * np.eye(3)
        Q[3:, 3:] = (self._sigma_bias_rw ** 2) * dt * np.eye(3)

        self._P = F @ self._P @ F.T + Q

    # --- private: update helpers ---
    def _update_accel(self, accel: np.ndarray) -> None:
        """
        중력 기준벡터로 attitude error 업데이트.
        NED specific force: h = R^T · [0,0,-g] (up direction, accel이 측정하는 방향)
        """
        g_world = np.array([0.0, 0.0, -self._g])
        h = self._rotate_world_to_body(self._q, g_world)

        # Jacobian H (3×6): dh/d[δθ, δb]
        H = np.zeros((3, 6))
        H[:3, :3] = self._skew(h)  # dh/dδθ = [h×]

        R_noise = (self._sigma_accel ** 2) * np.eye(3)
        self._ekf_update(accel, h, H, R_noise)

    def _update_gyro(self, gyro: np.ndarray) -> None:
        """
        자이로 측정 모델: h = bias (gyro bias 관측)
        Jacobian H (3×6): dh/d[δθ, δb] = [0, I]
        """
        h = self._bias.copy()

        H = np.zeros((3, 6))
        H[:3, 3:] = np.eye(3)

        R_noise = (self._sigma_bias_rw ** 2) * np.eye(3)
        self._ekf_update(gyro, h, H, R_noise)

    def _update_mag(self, mag: np.ndarray) -> None:
        """
        자기장 기준벡터로 heading error 업데이트.
        측정 모델: h = R^T · m_world
        """
        h = self._rotate_world_to_body(self._q, self._mag_ref)

        H = np.zeros((3, 6))
        H[:3, :3] = self._skew(h)

        R_noise = (self._sigma_mag ** 2) * np.eye(3)
        self._ekf_update(mag, h, H, R_noise)

    def _ekf_update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))

        innov = z - h
        dx = K @ innov  # error state

        # error injection
        dtheta = dx[:3]
        dbias  = dx[3:]

        # small angle perturbation apply: q ← q ⊗ [dθ/2, 1] (normalized)
        dq = np.array([dtheta[0]/2, dtheta[1]/2, dtheta[2]/2, 1.0])
        self._q = Transforms.quat_normalize(
            Transforms.quat_mult(self._q, dq / np.linalg.norm(dq))
        )
        self._bias += dbias

        # Joseph form for numerical stability
        I_KH = np.eye(6) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T

    # --- static helpers ---
    @staticmethod
    def _rotate_world_to_body(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """world → body: q* ⊗ [v,0] ⊗ q"""
        qv = np.array([v[0], v[1], v[2], 0.0])
        r = Transforms.quat_mult(
            Transforms.quat_mult(Transforms.quat_conjugate(q), qv), q
        )
        return r[:3]

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        return np.array([
            [ 0,    -v[2],  v[1]],
            [ v[2],  0,    -v[0]],
            [-v[1],  v[0],  0   ],
        ], dtype=float)