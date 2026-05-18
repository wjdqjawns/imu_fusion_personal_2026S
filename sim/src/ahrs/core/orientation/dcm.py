"""
File Name: ./src/core/orientation/dcm.py
Author: Beomjun Chung
Updated: 2026-05-16

Description:
    Direction Cosine Matrix (DCM) 클래스

    역할:
        벡터 좌표계 변환 담당.
        중력 벡터, 자기장 벡터를 body ↔ world 사이에서 변환할 때 주로 사용.
        EKF Jacobian 계산에서도 자주 등장.

        필터 내부 상태로는 quaternion을 유지하다가
        벡터 변환이 필요한 순간에만 DCM으로 변환하는 패턴이 일반적.

    직교성 조건:
        R · R^T = I, det(R) = +1
        적분 중 이 조건이 깨지면 Gram-Schmidt 정규화로 복원.
        (쿼터니언 적분보다 이 문제가 심해서 DCM 적분은 잘 안 씀)

    좌표 변환:
        v_world = R · v_body   (body → world)
        v_body  = R^T · v_world  (world → body)
"""

from __future__ import annotations

import numpy as np

from .transforms import Transforms

class DCM:
    def __init__(self, R: np.ndarray | None = None):
        if R is None: # no rotation
            self._R = np.eye(3, dtype=float)
        else:
            self._R = np.asarray(R, dtype=float)
            assert self._R.shape == (3, 3), "DCM must be 3×3"

    @classmethod
    def identity(cls) -> "DCM":
        return cls(np.eye(3))

    @classmethod
    def from_euler(cls, roll: float, pitch: float, yaw: float) -> "DCM":
        """Euler [rad] (ZYX)에서 생성."""
        return cls(Transforms.euler_to_dcm(roll, pitch, yaw))

    @classmethod
    def from_quaternion(cls, q: np.ndarray) -> "DCM":
        """쿼터니언 [x,y,z,w] (scalar-last)에서 생성."""
        return cls(Transforms.quat_to_dcm(q))

    @classmethod
    def from_axis_angle(cls, axis: np.ndarray, angle_rad: float) -> "DCM":
        """
        Rodrigues 공식: 축-각도 → DCM.
        R = I + sin(θ)·[n]× + (1-cos(θ))·[n]×²
        """
        axis = np.asarray(axis, dtype=float)
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        K = cls._skew(axis)
        R = (np.eye(3)
             + np.sin(angle_rad) * K
             + (1 - np.cos(angle_rad)) * K @ K)
        return cls(R)

    @property
    def R(self) -> np.ndarray:
        return self._R.copy()

    @property
    def T(self) -> "DCM":
        """전치 = 역 (직교행렬이므로)."""
        return DCM(self._R.T)

    def __matmul__(self, other: "DCM") -> "DCM":
        """회전 합성: self · other."""
        return DCM(self._R @ other._R)

    def __repr__(self) -> str:
        return f"DCM(\n{np.round(self._R, 4)}\n)"

    def normalize(self) -> "DCM":
        """
        Gram-Schmidt 직교화로 직교성 복원.
        수치 오차로 인해 R·R^T ≠ I가 됐을 때 사용.

        알고리즘:
            x_new = x / |x|
            y_new = (y - (y·x̂)x̂) / |...|   (x에 직교화)
            z_new = x̂ × ŷ
        """
        x = self._R[:, 0]
        y = self._R[:, 1]

        x = x / (np.linalg.norm(x) + 1e-12)
        y = y - np.dot(y, x) * x
        y = y / (np.linalg.norm(y) + 1e-12)
        z = np.cross(x, y)

        return DCM(np.column_stack([x, y, z]))

    def is_valid(self, tol: float = 1e-6) -> bool:
        """직교성과 행렬식 조건 확인."""
        ortho = np.allclose(self._R @ self._R.T, np.eye(3), atol=tol)
        det   = abs(np.linalg.det(self._R) - 1.0) < tol
        return ortho and det

    def rotate(self, v: np.ndarray) -> np.ndarray:
        """body → world: v_world = R · v_body"""
        return self._R @ np.asarray(v, dtype=float)

    def rotate_inv(self, v: np.ndarray) -> np.ndarray:
        """world → body: v_body = R^T · v_world"""
        return self._R.T @ np.asarray(v, dtype=float)

    def to_quaternion(self) -> np.ndarray:
        """DCM → 쿼터니언 [x,y,z,w] (scalar-last)."""
        return Transforms.dcm_to_quat(self._R)

    def to_euler(self) -> np.ndarray:
        """DCM → Euler [roll, pitch, yaw] [rad]."""
        return Transforms.dcm_to_euler(self._R)

    def to_euler_deg(self) -> np.ndarray:
        return np.degrees(self.to_euler())

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        """
        Skew-symmetric(반대칭) 행렬 [v]×.
        cross product를 행렬 곱으로 표현: [v]× · u = v × u

        EKF Jacobian, Rodrigues 공식에서 사용.
        """
        return np.array([
            [ 0,    -v[2],  v[1]],
            [ v[2],  0,    -v[0]],
            [-v[1],  v[0],  0   ],
        ], dtype=float)

    @staticmethod
    def skew(v: np.ndarray) -> np.ndarray:
        """Public wrapper for skew-symmetric matrix."""
        return DCM._skew(v)