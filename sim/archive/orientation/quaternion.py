"""
File Name: ./src/ahrs/core/orientation/quaternion.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Quaternion 클래스 — 필터 내부 상태 유지, 자이로 적분, 회전 합성 담당

    purpose:
        scalar-last [x, y, z, w] 컨벤션으로 quaternion 연산 전반을 캡슐화.
        변환 함수(to_dcm, to_euler)는 Transforms 위임.
        연산(곱셈, 켤레, 정규화)은 직접 구현.

    Notes:
        - Identity: [0, 0, 0, 1]
        - q와 -q는 동일 회전 (double cover)
        - EKF 오차 계산 시 ensure_positive_w()로 부호 통일 권장
        - 적분 후 매 스텝 normalize() 수행
"""

from __future__ import annotations

import numpy as np

from ahrs.utils.transforms import Transforms


class Quaternion:
    __slots__ = ("_q",)

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 z: float = 0.0, w: float = 1.0):
        """scalar-last: q = [x, y, z, w]"""
        self._q = np.array([x, y, z, w], dtype=float)

    @classmethod
    def identity(cls) -> "Quaternion":
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_array(cls, q: np.ndarray) -> "Quaternion":
        q = Transforms.quat_normalize(np.asarray(q, dtype=float))
        return cls(*q)

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> "Quaternion":
        return cls.from_array(Transforms.euler_to_quat(roll, pitch, yaw))

    @classmethod
    def from_dcm(cls, R: np.ndarray) -> "Quaternion":
        return cls.from_array(Transforms.dcm_to_quat(R))

    @classmethod
    def from_axis_angle(cls, axis: np.ndarray, angle_rad: float) -> "Quaternion":
        axis = np.asarray(axis, dtype=float)
        n = np.linalg.norm(axis)
        if n < 1e-10:
            return cls.identity()
        axis = axis / n
        half = angle_rad / 2.0
        return cls(
            axis[0] * np.sin(half),
            axis[1] * np.sin(half),
            axis[2] * np.sin(half),
            np.cos(half),
        )

    @property
    def q(self) -> np.ndarray:
        return self._q.copy()

    @property
    def x(self) -> float: return float(self._q[0])

    @property
    def y(self) -> float: return float(self._q[1])

    @property
    def z(self) -> float: return float(self._q[2])

    @property
    def w(self) -> float: return float(self._q[3])

    @property
    def norm(self) -> float:
        return float(np.linalg.norm(self._q))

    @property
    def vec(self) -> np.ndarray:
        """벡터 부분 [x, y, z]."""
        return self._q[:3].copy()

    def normalize(self) -> "Quaternion":
        return Quaternion.from_array(self._q)

    def conjugate(self) -> "Quaternion":
        """켤레 q* = [-x, -y, -z, w]."""
        return Quaternion(-self._q[0], -self._q[1], -self._q[2], self._q[3])

    def inverse(self) -> "Quaternion":
        return self.conjugate().normalize()

    def ensure_positive_w(self) -> "Quaternion":
        """w < 0 이면 부호 반전. EKF 오차 연속성 보장."""
        if self._q[3] < 0:
            return Quaternion.from_array(-self._q)
        return Quaternion.from_array(self._q)

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        """Hamilton product: self ⊗ other."""
        return Quaternion.from_array(Transforms.quat_mult(self._q, other._q))

    def __neg__(self) -> "Quaternion":
        return Quaternion.from_array(-self._q)

    def __repr__(self) -> str:
        return (f"Quaternion(x={self.x:.4f}, y={self.y:.4f}, "
                f"z={self.z:.4f}, w={self.w:.4f})")

    def rotate(self, v: np.ndarray) -> np.ndarray:
        """
        벡터 v를 이 쿼터니언으로 회전.
        v_out = q ⊗ [vx, vy, vz, 0] ⊗ q*   (body → world)
        pure quaternion: [vx, vy, vz, 0] in [x,y,z,w] form
        """
        v = np.asarray(v, dtype=float)
        qv = np.array([v[0], v[1], v[2], 0.0])
        result = Transforms.quat_mult(
            Transforms.quat_mult(self._q, qv),
            Transforms.quat_conjugate(self._q)
        )
        return result[:3]

    def integrate(self, omega: np.ndarray, dt: float,
                  method: str = "euler") -> "Quaternion":
        """
        자이로 적분 디스패처.
        method: "euler" | "rk4" | "exact"
        """
        if method == "euler":
            return self._integrate_euler(omega, dt)
        elif method == "rk4":
            return self._integrate_rk4(omega, dt)
        elif method == "exact":
            return self._integrate_exact(omega, dt)
        else:
            raise ValueError(f"Unknown method: {method}")

    def _integrate_euler(self, omega: np.ndarray, dt: float) -> "Quaternion":
        """
        1차 오일러 적분.
        q̇ = ½·q⊗Ω, Ω = [ωx, ωy, ωz, 0] (scalar-last pure quat)
        100Hz 이상에서 충분히 정확.
        """
        Omega = np.array([omega[0], omega[1], omega[2], 0.0])
        q_dot = 0.5 * Transforms.quat_mult(self._q, Omega)
        q_new = self._q + q_dot * dt
        return Quaternion.from_array(q_new)

    def _integrate_rk4(self, omega: np.ndarray, dt: float) -> "Quaternion":
        """4차 Runge-Kutta 적분. ω가 dt 동안 일정하다고 가정."""
        def qdot(q: np.ndarray) -> np.ndarray:
            Omega = np.array([omega[0], omega[1], omega[2], 0.0])
            return 0.5 * Transforms.quat_mult(q, Omega)

        k1 = qdot(self._q)
        k2 = qdot(self._q + 0.5*dt*k1)
        k3 = qdot(self._q + 0.5*dt*k2)
        k4 = qdot(self._q + dt*k3)
        q_new = self._q + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return Quaternion.from_array(q_new)

    def _integrate_exact(self, omega: np.ndarray, dt: float) -> "Quaternion":
        """
        지수맵(Exponential Map) 적분.
        q(t+dt) = q(t) ⊗ exp(Ω·dt/2)
        ω가 dt 동안 일정할 때 이론적으로 정확.
        """
        angle = np.linalg.norm(omega) * dt
        if angle < 1e-10:
            return Quaternion.from_array(self._q)
        axis = omega / np.linalg.norm(omega)
        dq = Quaternion.from_axis_angle(axis, angle)
        return Quaternion.from_array(
            Transforms.quat_mult(self._q, dq._q)
        )

    @staticmethod
    def slerp(q0: "Quaternion", q1: "Quaternion", t: float) -> "Quaternion":
        """
        Spherical Linear Interpolation.
        t=0 → q0, t=1 → q1.
        """
        a, b = q0._q, q1._q
        dot = np.clip(np.dot(a, b), -1.0, 1.0)
        if dot < 0:
            b = -b
            dot = -dot
        if dot > 0.9995:
            return Quaternion.from_array(a + t * (b - a))
        theta_0 = np.arccos(dot)
        theta   = theta_0 * t
        sin_t0  = np.sin(theta_0)
        s0 = np.cos(theta) - dot * np.sin(theta) / sin_t0
        s1 = np.sin(theta) / sin_t0
        return Quaternion.from_array(s0*a + s1*b)

    # ── 변환 ──────────────────────────────────────────────────────────────────

    def to_dcm(self) -> np.ndarray:
        return Transforms.quat_to_dcm(self._q)

    def to_euler(self) -> np.ndarray:
        """[roll, pitch, yaw] [rad]. 출력 전용."""
        return Transforms.quat_to_euler(self._q)

    def to_euler_deg(self) -> np.ndarray:
        return np.degrees(self.to_euler())

    # ── 오차 계산 ─────────────────────────────────────────────────────────────

    def error_from(self, q_true: "Quaternion") -> "Quaternion":
        """오차 쿼터니언: q_err = q_true* ⊗ self."""
        return Quaternion.from_array(
            Transforms.quat_error(self._q, q_true._q)
        )

    def geodesic_error(self, q_true: "Quaternion") -> float:
        """진짜 각도 오차 [rad]. 짐벌락 영향 없음."""
        return Transforms.geodesic_error(self._q, q_true._q)
