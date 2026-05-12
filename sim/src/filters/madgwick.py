import numpy as np
from src.base import AhrsFilter
from src.utils.quaternion import q_normalize, q_rotate, integrate_quaternion, normalize, q_conj


class MadgwickFilter(AhrsFilter):
    name = "madgwick"

    def __init__(self, beta=0.08):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.bias = np.zeros(3, dtype=float)

    @property
    def quaternion(self):
        return self.q.copy()

    def update(self, gyro, accel, mag, dt):
        a = normalize(accel)
        m = normalize(mag)

        g_pred = q_rotate(q_conj(self.q), np.array([0.0, 0.0, -1.0]))
        m_ref_world = normalize(np.array([0.45, 0.0, -0.2], dtype=float))
        m_pred = q_rotate(q_conj(self.q), m_ref_world)

        e = np.cross(g_pred, a) + 0.5 * np.cross(m_pred, m)
        omega_corr = gyro + self.beta * e
        self.q = q_normalize(integrate_quaternion(self.q, omega_corr, dt))
        return self.q.copy(), self.bias.copy()
