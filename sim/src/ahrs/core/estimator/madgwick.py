"""
File Name: ./src/ahrs/core/filters/madgwick.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Madgwick Filter — gradient descent 기반 AHRS 필터

    purpose:
        Error function Slope compensation
        \beta = \omega_{err} \cdot \sqrt{\frac{3}{4}}: 자이로 측정 오차 가중치.

    Notes:
        NED 프레임 중력 컨벤션:
            가속도계는 specific force = R^T·[0,0,-g] 출력.
            Madgwick의 목적 함수는 gravity direction = +z 기준 설계이므로
            accel 입력을 부호 반전: â_gravity = -a_meas_normalized
            → 올바른 objective: f_g = body_gravity_pred - â_gravity

        mag 컨벤션:
            tilt-compensated magnetic heading 방식 사용.
            body frame에서 측정된 mag을 world로 투영 후 수평 성분만 사용.
"""

from __future__ import annotations

import numpy as np

from ahrs.core.estimator.base import AhrsFilter
from ahrs.utils.transforms import Transforms

class MadgwickFilter(AhrsFilter):
    """Madgwick AHRS Filter."""
    name = "madgwick"

    def __init__(self, beta: float = 0.08,
                 use_magnetometer: bool = True, **kwargs):
        self._beta    = float(beta)
        self._use_mag = bool(use_magnetometer)
        self._q       = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias    = np.zeros(3, dtype=float)

    @property
    def quaternion(self) -> np.ndarray:
        return self._q.copy()

    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        q = self._q
        x, y, z, w = q

        grad = np.zeros(4, dtype=float)

        # --- gradient from accel ---
        a_norm = np.linalg.norm(accel)
        if a_norm > 1e-6:
            # NED specific force -> gravity direction (negate)
            # \hat{a} = -accel_normalized (gravity direction in body frame)
            ax, ay, az = -accel / a_norm

            # Objective: f_g = q^* \kron [0,0,1,0] \kron q - \hat{a}
            # body prediction [x,y,z,w] = [2(xz-wy), 2(wx+yz), 1-2x²-2y²]
            f1 = 2*(x*z - w*y) - ax
            f2 = 2*(w*x + y*z) - ay
            f3 = 2*(0.5 - x**2 - y**2) - az

            # Gradient: J^T · f  (Jacobian of f_g w.r.t. [x,y,z,w])
            # df1/d[x,y,z,w] = [2z, -2w, 2x, -2y]
            # df2/d[x,y,z,w] = [2w, 2z, 2y, 2x]  ← wait
            # Let me compute carefully
            # f1 = 2xz - 2wy - ax  → df1/dx=2z, df1/dy=-2w? No.
            # Actually f1 = 2*(x*z) - 2*(w*y) - ax
            # df1/dx = 2z, df1/dy = -2w? No: -2w*dy/dy = 0 → wait.
            # Let me be careful: x,y,z,w are components of quaternion, w is scalar last.
            # f1 = 2*(x*z - w*y) - ax
            # df1/dx = 2z, df1/dy = -2w, df1/dz = 2x, df1/dw = -2y
            # f2 = 2*(w*x + y*z) - ay
            # df2/dx = 2w, df2/dy = 2z, df2/dz = 2y, df2/dw = 2x
            # f3 = 2*(0.5 - x²- y²) - az = 1 - 2x² - 2y² - az
            # df3/dx = -4x, df3/dy = -4y, df3/dz = 0, df3/dw = 0

            g_acc = np.array([
                2*z*f1  + 2*w*f2 - 4*x*f3,   # dF/dx
               -2*w*f1  + 2*z*f2 - 4*y*f3,   # dF/dy
                2*x*f1  + 2*y*f2,             # dF/dz
               -2*y*f1  + 2*x*f2,             # dF/dw
            ])
            grad += g_acc

        # --- gradient from mag ---
        if self._use_mag:
            m_norm = np.linalg.norm(mag)
            if m_norm > 1e-6:
                mx, my, mz = mag / m_norm

                # Rotate mag to world frame, flatten to horizontal (tilt compensation)
                # h = q ⊗ [mx,my,mz,0] ⊗ q*
                qv = np.array([mx, my, mz, 0.0])
                h_world = Transforms.quat_mult(
                    Transforms.quat_mult(q, qv), Transforms.quat_conjugate(q)
                )[:3]
                bx = np.sqrt(h_world[0]**2 + h_world[1]**2)  # horizontal magnitude
                bz = h_world[2]

                # Objective: f_b = q* ⊗ [bx,0,bz,0] ⊗ q - m_hat
                # Body prediction for b=[bx,0,bz]:
                # [x,y,z,w] components:
                f4 = 2*bx*(0.5 - y**2 - z**2) + 2*bz*(x*z - w*y) - mx
                f5 = 2*bx*(x*y - w*z)         + 2*bz*(w*x + y*z) - my
                f6 = 2*bx*(w*y + x*z)         + 2*bz*(0.5 - x**2 - y**2) - mz

                # Gradient: J_b^T · f_b
                g_mag = np.array([
                    2*bz*z*f4              + (2*bx*y + 2*bz*w)*f5  + (2*bx*z - 4*bz*x)*f6,
                    (-4*bx*y - 2*bz*w)*f4 + (2*bx*x + 2*bz*z)*f5  + (2*bx*w - 4*bz*y)*f6,
                    (2*bz*x)*f4            + (-2*bx*w + 2*bz*y)*f5 + (2*bx*x)*f6,
                    (-2*bz*y)*f4           + (-2*bx*z + 2*bz*x)*f5 + (2*bx*y)*f6,
                ])
                grad += g_mag

        # --- gyro integral (gradient descent compensation) ---
        grad_norm = np.linalg.norm(grad)
        if grad_norm > 1e-10:
            grad = grad / grad_norm

        Omega = np.array([gyro[0], gyro[1], gyro[2], 0.0])
        q_dot_gyro = 0.5 * Transforms.quat_mult(q, Omega)
        q_new = q + (q_dot_gyro - self._beta * grad) * dt
        self._q = Transforms.quat_normalize(q_new)

        return self._q.copy(), self._bias.copy()

    def reset(self) -> None:
        self._q    = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)