import numpy as np
from src.base import AhrsFilter
from src.utils.quaternion import q_normalize, q_rotate, integrate_quaternion, normalize, q_conj


class MahonyFilter(AhrsFilter):
    name = "mahony"

    def __init__(self, kp=1.8, ki=0.08):
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.bias = np.zeros(3, dtype=float)
        self.int_err = np.zeros(3, dtype=float)

    @property
    def quaternion(self):
        return self.q.copy()

    def update(self, gyro, accel, mag, dt):
        a = normalize(accel)
        m = normalize(mag)

        g_pred = q_rotate(q_conj(self.q), np.array([0.0, 0.0, -1.0]))
        m_ref_world = normalize(np.array([0.45, 0.0, -0.2], dtype=float))
        m_pred = q_rotate(q_conj(self.q), m_ref_world)

        e_acc = np.cross(g_pred, a)
        e_mag = np.cross(m_pred, m)
        e = e_acc + 0.5 * e_mag

        self.int_err += e * dt
        self.bias += -self.ki * e * dt
        omega = gyro - self.bias + self.kp * e

        self.q = q_normalize(integrate_quaternion(self.q, omega, dt))
        return self.q.copy(), self.bias.copy()
