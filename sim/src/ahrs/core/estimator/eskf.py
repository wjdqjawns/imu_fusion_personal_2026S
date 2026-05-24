"""
File Name: ./src/ahrs/core/estimator/eskf.py
Author: Beomjun Chung
Updated: 2026-05-24

Description:
    ESKF (Error-State Kalman Filter) — 실용 AHRS 표준

    Two-layer state:
        nominal : q̂ (scalar-last [x,y,z,w]), b̂_g (3,)
        error   : δx = [δθ (3), δb_g (3)]  — 6-dim Euclidean

    Predict:
        q̂ ← q̂ ⊗ Exp((ω̃ - b̂_g)·dt)
        F  = error dynamics Jacobian (6×6)
        P  ← F P Fᵀ + Q

    Update (accel then mag, sequential):
        H  = tangent-space Jacobian of h(x̂) w.r.t. δx
        K  = P Hᵀ (H P Hᵀ + R)⁻¹
        δx = K (z - h(x̂))

    Inject & Reset (manifold composition):
        q̂   ← normalize(q̂ ⊗ Exp(δθ/2))
        b̂_g ← b̂_g + δb_g
        P   ← (I - KH) P
        δx  ← 0

    Returns: (q̂, b̂_g)
"""

from __future__ import annotations

import numpy as np

from ahrs.core.estimator.base import AhrsFilter
from ahrs.utils.transforms import Transforms


class EskfFilter(AhrsFilter):
    name = "eskf"

    def __init__(
        self,
        sigma_gyro: float = 0.001,
        sigma_accel: float = 0.01,
        sigma_mag: float = 0.5,
        sigma_gyro_bias: float = 0.0001,
        init_attitude_std_deg: float = 10.0,
        init_bias_std_rad_s: float = 0.01,
        gravity_m_s2: float = 9.80665,
        mag_ref_uT: np.ndarray | None = None,
        accel_threshold_g: float = 0.2,
        **kwargs,
    ):
        self._sg    = float(sigma_gyro)
        self._sa    = float(sigma_accel)
        self._sm    = float(sigma_mag)
        self._sb    = float(sigma_gyro_bias)
        self._g     = float(gravity_m_s2)
        self._a_thr = float(accel_threshold_g) * self._g

        # nominal state
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)

        # error state covariance P ∈ ℝ⁶ˣ⁶
        att_var  = np.deg2rad(init_attitude_std_deg) ** 2
        bias_var = float(init_bias_std_rad_s) ** 2
        self._P = np.block([
            [att_var  * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)),     bias_var * np.eye(3)],
        ])

        self._mag_ref: np.ndarray | None = (
            np.asarray(mag_ref_uT, dtype=float) if mag_ref_uT is not None else None
        )
        self._mag_ref_norm: float = (
            float(np.linalg.norm(mag_ref_uT)) if mag_ref_uT is not None else 0.0
        )

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def reset(self) -> None:
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)
        att_var = np.deg2rad(10.0) ** 2
        self._P = np.block([
            [att_var * np.eye(3),  np.zeros((3, 3))],
            [np.zeros((3, 3)),     0.01**2 * np.eye(3)],
        ])

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        # ── Predict ───────────────────────────────────────────────────────────
        omega = gyro - self._bias

        # propagate nominal quaternion
        dq = Transforms.rotvec_to_quat(omega * dt)
        self._q = Transforms.quat_normalize(Transforms.quat_mult(self._q, dq))

        # error dynamics Jacobian F (6×6)
        # δθ̇ = -[ω×]δθ - δb,  δḃ = 0
        F = np.zeros((6, 6))
        F[:3, :3] = -self._skew(omega)
        F[:3, 3:] = -np.eye(3)
        Fdt = np.eye(6) + F * dt

        Q = np.zeros((6, 6))
        Q[:3, :3] = (self._sg ** 2) * dt * np.eye(3)
        Q[3:, 3:] = (self._sb ** 2) * dt * np.eye(3)

        self._P = Fdt @ self._P @ Fdt.T + Q

        # ── Update: accel ──────────────────────────────────────────────────────
        if abs(np.linalg.norm(accel) - self._g) < self._a_thr:
            self._update_accel(accel)

        # ── Update: mag ────────────────────────────────────────────────────────
        if self._mag_ref is None and np.linalg.norm(mag) > 1e-6:
            self._mag_ref = Transforms.body2world(self._q, mag)
            self._mag_ref_norm = float(np.linalg.norm(self._mag_ref))

        if self._mag_ref is not None and self._mag_ref_norm > 1e-6:
            mag_norm = np.linalg.norm(mag)
            if abs(mag_norm - self._mag_ref_norm) < 0.3 * self._mag_ref_norm:
                self._update_mag(mag)

        return self._q.copy(), self._bias.copy()

    # ── private update helpers ─────────────────────────────────────────────────
    def _update_accel(self, accel: np.ndarray) -> None:
        """
        Specific force observation in body frame.
        h(x̂) = R(q̂)^T · [0, 0, -g]  (NED: gravity=[0,0,+g], specific force=-gravity)
        H = [h×] | 0  (3×6, tangent-space Jacobian)
        """
        g_world = np.array([0.0, 0.0, -self._g])  # specific force reference in world
        h = Transforms.world2body(self._q, g_world)

        # H[:3,:3] = skew(h) (rotation error Jacobian)
        H = np.zeros((3, 6))
        H[:3, :3] = self._skew(h)

        R_n = (self._sa ** 2) * np.eye(3)
        self._eskf_update(accel, h, H, R_n)

    def _update_mag(self, mag: np.ndarray) -> None:
        """
        Magnetometer observation in body frame.
        h(x̂) = R(q̂)^T · m_world
        H = [h×] | 0
        """
        h = Transforms.world2body(self._q, self._mag_ref)

        H = np.zeros((3, 6))
        H[:3, :3] = self._skew(h)

        R_n = (self._sm ** 2) * np.eye(3)
        self._eskf_update(mag, h, H, R_n)

    def _eskf_update(self, z: np.ndarray, h: np.ndarray,
                     H: np.ndarray, R: np.ndarray) -> None:
        """Generic ESKF measurement update + inject + reset."""
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))

        dx = K @ (z - h)
        dtheta = dx[:3]
        dbias  = dx[3:]

        # inject: q̂ ← normalize(q̂ ⊗ Exp(δθ/2))
        dq = Transforms.rotvec_to_quat(dtheta)
        self._q    = Transforms.quat_normalize(Transforms.quat_mult(self._q, dq))
        self._bias = self._bias + dbias

        # reset P (Joseph form)
        I_KH = np.eye(6) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        return np.array([
            [ 0.0,  -v[2],  v[1]],
            [ v[2],  0.0,  -v[0]],
            [-v[1],  v[0],  0.0 ],
        ], dtype=float)
