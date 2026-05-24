"""
File Name: ./src/ahrs/core/estimator/ekf_euler.py
Author: Beomjun Chung
Updated: 2026-05-24

Description:
    EKF (Euler angle state) — 교육용 기준선 필터

    state: x = [φ, θ, ψ, b_gx, b_gy, b_gz]^T  (6×1, rad)

    Process model:
        ẋ_att = W(φ,θ) · (ω_meas - b_g)
        W =  [1  sinφ·tanθ    cosφ·tanθ ]
             [0    cosφ        -sinφ     ]
             [0  sinφ/cosθ   cosφ/cosθ  ]

    Singularity at |cosθ| < 1e-4 (pitch ±90°) → W clamped, filter diverges
    (교육 목적: 특이점 시각화용)

    Observation:
        accel → gravity direction → roll/pitch correction
        mag   → heading           → yaw correction

    Returns: (q_from_euler [x,y,z,w], b_g)
"""

from __future__ import annotations

import numpy as np

from ahrs.core.estimator.base import AhrsFilter
from ahrs.utils.transforms import Transforms


class EkfEulerFilter(AhrsFilter):
    name = "ekf_euler"

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
        self._sg     = float(sigma_gyro)
        self._sa     = float(sigma_accel)
        self._sm     = float(sigma_mag)
        self._sb     = float(sigma_gyro_bias)
        self._g      = float(gravity_m_s2)
        self._a_thr  = float(accel_threshold_g) * self._g

        # state: [roll, pitch, yaw, bgx, bgy, bgz]
        self._x = np.zeros(6, dtype=float)

        # covariance 6×6
        att_var  = np.deg2rad(init_attitude_std_deg) ** 2
        bias_var = float(init_bias_std_rad_s) ** 2
        self._P = np.block([
            [att_var  * np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)),     bias_var * np.eye(3)],
        ])

        self._mag_ref: np.ndarray | None = (
            np.asarray(mag_ref_uT, dtype=float) if mag_ref_uT is not None else None
        )

        # cache for property
        self._q = Transforms.euler_to_quat(0.0, 0.0, 0.0)
        self._bias = np.zeros(3, dtype=float)

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def reset(self) -> None:
        self._x[:] = 0.0
        att_var = np.deg2rad(10.0) ** 2
        self._P = np.block([
            [att_var * np.eye(3),  np.zeros((3, 3))],
            [np.zeros((3, 3)),     0.01**2 * np.eye(3)],
        ])
        self._q = Transforms.euler_to_quat(0.0, 0.0, 0.0)
        self._bias = np.zeros(3, dtype=float)

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        phi, theta, psi = self._x[:3]
        bias = self._x[3:]

        # ── Predict ───────────────────────────────────────────────────────────
        omega = gyro - bias

        cos_theta = np.cos(theta)
        # singularity guard: clamp cos(θ) to avoid divide-by-zero
        if abs(cos_theta) < 1e-4:
            cos_theta = np.sign(cos_theta) * 1e-4

        tan_theta = np.sin(theta) / cos_theta
        sin_phi   = np.sin(phi)
        cos_phi   = np.cos(phi)

        # W matrix: Euler kinematics
        W = np.array([
            [1.0,  sin_phi * tan_theta,   cos_phi * tan_theta],
            [0.0,  cos_phi,              -sin_phi            ],
            [0.0,  sin_phi / cos_theta,   cos_phi / cos_theta],
        ])

        x_dot_att = W @ omega
        self._x[:3] += x_dot_att * dt
        # bias random walk stays constant in predict

        # ── Jacobian F (6×6) ─────────────────────────────────────────────────
        # ∂ẋ_att / ∂φ, ∂ẋ_att / ∂θ — analytical
        p, q_w, r = omega  # body rates after bias removal

        dW_dphi = np.array([
            [0.0,  cos_phi * tan_theta,  -sin_phi * tan_theta],
            [0.0, -sin_phi,              -cos_phi            ],
            [0.0,  cos_phi / cos_theta,  -sin_phi / cos_theta],
        ])

        sec2_theta  = 1.0 / (cos_theta ** 2)
        dW_dtheta = np.array([
            [0.0,  sin_phi * sec2_theta,   cos_phi * sec2_theta],
            [0.0,  0.0,                    0.0                 ],
            [0.0,  sin_phi * np.sin(theta) / (cos_theta**2),
                   cos_phi * np.sin(theta) / (cos_theta**2)],
        ])

        F = np.zeros((6, 6))
        F[:3, 0] = dW_dphi   @ omega   # ∂ẋ/∂φ
        F[:3, 1] = dW_dtheta @ omega   # ∂ẋ/∂θ
        # ∂ẋ/∂ψ = 0 (W doesn't depend on ψ)
        F[:3, 3:] = -W                 # ∂ẋ/∂b = -W

        Fdt = np.eye(6) + F * dt

        # process noise
        Q = np.zeros((6, 6))
        Q[:3, :3] = (self._sg ** 2) * dt * np.eye(3)
        Q[3:, 3:] = (self._sb ** 2) * dt * np.eye(3)

        self._P = Fdt @ self._P @ Fdt.T + Q

        # ── Update: accel → roll/pitch ────────────────────────────────────────
        if abs(np.linalg.norm(accel) - self._g) < self._a_thr:
            self._update_accel(accel)

        # ── Update: mag → yaw ────────────────────────────────────────────────
        if self._mag_ref is None and np.linalg.norm(mag) > 1e-6:
            # initialize from first measurement projected to world
            q_init = Transforms.euler_to_quat(*self._x[:3])
            R = Transforms.quat_to_dcm(q_init)
            self._mag_ref = R @ mag

        if self._mag_ref is not None and np.linalg.norm(mag) > 1e-6:
            self._update_mag(mag)

        # wrap angles to [-π, π]
        self._x[:3] = (self._x[:3] + np.pi) % (2 * np.pi) - np.pi

        # convert euler → quaternion for return
        self._q    = Transforms.euler_to_quat(*self._x[:3])
        self._bias = self._x[3:].copy()
        return self._q.copy(), self._bias.copy()

    # ── private update helpers ────────────────────────────────────────────────
    def _update_accel(self, accel: np.ndarray) -> None:
        """
        Accel → roll/pitch 관측.
        측정 모델: h = R(φ,θ)^T · [0, 0, -g]  (specific force, NED gravity=[0,0,+g])
        h = [sinθ, -cosθ·sinφ, -cosθ·cosφ] * g
        """
        phi, theta = self._x[0], self._x[1]
        g = self._g
        sin_phi, cos_phi   = np.sin(phi), np.cos(phi)
        sin_th,  cos_th    = np.sin(theta), np.cos(theta)

        # predicted specific force in body frame
        h = np.array([
             sin_th,
            -cos_th * sin_phi,
            -cos_th * cos_phi,
        ]) * g

        # H: 3×6  (∂h/∂[φ,θ,ψ,bx,by,bz])
        H = np.zeros((3, 6))
        H[0, 1] =  cos_th * g
        H[1, 0] = -cos_th * cos_phi * g
        H[1, 1] =  sin_th * sin_phi * g
        H[2, 0] =  cos_th * sin_phi * g
        H[2, 1] =  sin_th * cos_phi * g

        R_n = (self._sa ** 2) * np.eye(3)
        self._ekf_update(accel, h, H, R_n)

    def _update_mag(self, mag: np.ndarray) -> None:
        """
        Mag → yaw 관측.
        측정 모델: z = [mx, my, mz], h = R(φ,θ,ψ)^T · m_world
        H 수치 미분 (Jacobian 단순화).
        """
        eps = 1e-5
        q0  = Transforms.euler_to_quat(*self._x[:3])
        h   = Transforms.world2body(q0, self._mag_ref)

        H = np.zeros((3, 6))
        for j in range(3):
            xp = self._x.copy(); xp[j] += eps
            qp = Transforms.euler_to_quat(*xp[:3])
            hp = Transforms.world2body(qp, self._mag_ref)
            H[:, j] = (hp - h) / eps

        R_n = (self._sm ** 2) * np.eye(3)
        self._ekf_update(mag, h, H, R_n)

    def _ekf_update(self, z: np.ndarray, h: np.ndarray,
                    H: np.ndarray, R: np.ndarray) -> None:
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
        dx = K @ (z - h)
        self._x += dx
        I_KH = np.eye(6) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ R @ K.T
