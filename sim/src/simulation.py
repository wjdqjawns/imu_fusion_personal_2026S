from dataclasses import dataclass
import numpy as np
from src.utils.quaternion import integrate_quaternion, q_conj, q_mul, q_rotate, normalize


@dataclass
class SimulationData:
    t: np.ndarray
    q_true: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    mag: np.ndarray
    temp: np.ndarray
    euler_true: np.ndarray


def generate_truth(t: np.ndarray):
    n = len(t)
    dt = t[1] - t[0]
    q = np.zeros((n, 4), dtype=float)
    euler = np.zeros((n, 3), dtype=float)
    omega = np.zeros((n, 3), dtype=float)
    q[0] = np.array([1.0, 0.0, 0.0, 0.0])

    for k in range(n):
        tt = t[k]
        omega[k] = np.array([
            0.6 * np.sin(0.7 * tt),
            0.4 * np.cos(0.5 * tt),
            0.5 * np.sin(0.3 * tt + 0.4),
        ], dtype=float)
        if k > 0:
            q[k] = integrate_quaternion(q[k-1], omega[k-1], dt)
        else:
            q[k] = q[0]
    return q, omega


def simulate_imu(cfg: dict) -> SimulationData:
    dt = cfg["dt"]
    duration = cfg["duration"]
    t = np.arange(0.0, duration, dt)
    n = len(t)

    q_true, omega_true = generate_truth(t)

    g_world = np.array([0.0, 0.0, -1.0], dtype=float)
    m_world = normalize(np.array([0.45, 0.0, -0.2], dtype=float))

    gyro_bias = np.deg2rad(np.array(cfg["gyro_bias_deg_s"], dtype=float))
    gyro_noise_std = np.deg2rad(cfg["gyro_noise_std_deg_s"])
    accel_noise_std = cfg["accel_noise_std_g"]
    mag_noise_std = cfg["mag_noise_std"]
    hard_iron = np.array(cfg["hard_iron"], dtype=float)
    soft_iron = np.array(cfg["soft_iron"], dtype=float)

    temp = 25.0 + 5.0 * np.sin(0.05 * t)
    temp_bias = np.deg2rad(np.column_stack([
        0.02 * (temp - 25.0),
        -0.015 * (temp - 25.0),
        0.01 * (temp - 25.0),
    ]))

    gyro = np.zeros((n, 3), dtype=float)
    accel = np.zeros((n, 3), dtype=float)
    mag = np.zeros((n, 3), dtype=float)

    rng = np.random.default_rng(42)
    for k in range(n):
        q = q_true[k]
        gyro[k] = omega_true[k] + gyro_bias + temp_bias[k] + rng.normal(0.0, gyro_noise_std, 3)

        a_body = q_rotate(q_conj(q), g_world)
        accel[k] = a_body + rng.normal(0.0, accel_noise_std, 3)

        m_body = q_rotate(q_conj(q), m_world)
        mag[k] = soft_iron @ m_body + hard_iron + rng.normal(0.0, mag_noise_std, 3)

    euler_true = np.array([quat_to_euler_row(q) for q in q_true])
    return SimulationData(t=t, q_true=q_true, gyro=gyro, accel=accel, mag=mag, temp=temp, euler_true=euler_true)


def quat_to_euler_row(q):
    w, x, y, z = q
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1.0, 1.0))
    yaw = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return np.array([roll, pitch, yaw], dtype=float)