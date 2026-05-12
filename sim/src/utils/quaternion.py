import numpy as np

def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy()
    return v / n

def q_normalize(q: np.ndarray) -> np.ndarray:
    return normalize(q)

def q_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=float)

def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def q_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    qv = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    return q_mul(q_mul(q, qv), q_conj(q))[1:]

def q_from_omega(omega_body: np.ndarray, dt: float) -> np.ndarray:
    theta = np.linalg.norm(omega_body) * dt
    if theta < 1e-12:
        half = 0.5 * dt
        return q_normalize(np.array([1.0, half*omega_body[0], half*omega_body[1], half*omega_body[2]], dtype=float))
    axis = omega_body / np.linalg.norm(omega_body)
    half = 0.5 * theta
    return np.array([np.cos(half), *(np.sin(half) * axis)], dtype=float)

def integrate_quaternion(q: np.ndarray, omega_body: np.ndarray, dt: float) -> np.ndarray:
    dq = q_from_omega(omega_body, dt)
    return q_normalize(q_mul(q, dq))

def euler_from_quaternion(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w*y - z*x)
    pitch = np.sign(sinp) * np.pi/2 if abs(sinp) >= 1 else np.arcsin(sinp)

    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw], dtype=float)

def small_angle_quat(dtheta: np.ndarray) -> np.ndarray:
    return q_normalize(np.array([1.0, 0.5*dtheta[0], 0.5*dtheta[1], 0.5*dtheta[2]], dtype=float))