import numpy as np
from src.base import AhrsFilter
from src.utils.quaternion import (
    q_normalize,
    q_rotate,
    integrate_quaternion,
    normalize,
    small_angle_quat,
    q_mul,
    q_conj,
)


class EkfAhrs(AhrsFilter):
    name = "ekf"

    def __init__(
        self,
        gyro_noise_std_rad_s=0.01,
        gyro_bias_rw_std=0.0005,
        accel_meas_std=0.05,
        mag_meas_std=0.05,
    ):
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.bias = np.zeros(3, dtype=float)
        self.P = np.eye(6) * 0.05
        self.gyro_noise_std = gyro_noise_std_rad_s
        self.gyro_bias_rw_std = gyro_bias_rw_std
        self.accel_meas_std = accel_meas_std
        self.mag_meas_std = mag_meas_std
        self.m_ref_world = normalize(np.array([0.45, 0.0, -0.2], dtype=float))

    @property
    def quaternion(self):
        return self.q.copy()

    def predict(self, gyro, dt):
        omega = gyro - self.bias
        self.q = q_normalize(integrate_quaternion(self.q, omega, dt))

        F = np.eye(6)
        F[0:3, 3:6] = -np.eye(3) * dt
        qg = (self.gyro_noise_std ** 2) * dt * np.eye(3)
        qb = (self.gyro_bias_rw_std ** 2) * dt * np.eye(3)
        Q = np.block([[qg, np.zeros((3, 3))], [np.zeros((3, 3)), qb]])
        self.P = F @ self.P @ F.T + Q

    def measurement_model(self, q):
        g_pred = q_rotate(q_conj(q), np.array([0.0, 0.0, -1.0]))
        m_pred = q_rotate(q_conj(q), self.m_ref_world)
        return np.hstack([g_pred, m_pred])

    def measurement_jacobian(self, eps=1e-6):
        H = np.zeros((6, 6), dtype=float)
        h0 = self.measurement_model(self.q)
        for i in range(3):
            dtheta = np.zeros(3)
            dtheta[i] = eps
            dq = small_angle_quat(dtheta)
            q_pert = q_normalize(q_mul(self.q, dq))
            hi = self.measurement_model(q_pert)
            H[:, i] = (hi - h0) / eps
        return H

    def update(self, gyro, accel, mag, dt):
        self.predict(gyro, dt)

        z = np.hstack([normalize(accel), normalize(mag)])
        h = self.measurement_model(self.q)
        H = self.measurement_jacobian()

        R = np.diag([
            self.accel_meas_std**2,
            self.accel_meas_std**2,
            self.accel_meas_std**2,
            self.mag_meas_std**2,
            self.mag_meas_std**2,
            self.mag_meas_std**2,
        ])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ (z - h)
        self.P = (np.eye(6) - K @ H) @ self.P

        dtheta = dx[0:3]
        dbias = dx[3:6]
        self.q = q_normalize(q_mul(self.q, small_angle_quat(dtheta)))
        self.bias += dbias
        return self.q.copy(), self.bias.copy()
