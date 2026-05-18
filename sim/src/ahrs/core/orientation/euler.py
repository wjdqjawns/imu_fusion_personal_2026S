"""
File Name: ./src/core/orientation/euler.py
Author: Beomjun Chung
Updated: 2026-05-16

Description:
  Euler angles 클래스

    purpose:
        결과 저장, 플롯, 사람이 읽는 출력 전용.
        필터 내부에서는 절대 Euler를 상태로 유지하면 안 됨.

    Gimbal Lock:
        pitch = ±90° 일 때 roll과 yaw 축이 겹쳐 자유도를 잃음.
        - to_quaternion(), to_dcm() 은 짐벌락 근처에서도 잘 동작
        - from_quaternion(), from_dcm()은 짐벌락 근처에서 roll/yaw 값이 불안정
        - is_near_gimbal_lock()으로 감지 가능

    angle domain:
        general  : [-\pi, \pi] (arctan2 range)
        roll     : [-\pi ,   \pi]   (±\pi)
        pitch    : [-\pi/2, \pi/2] (±\pi/2)
        yaw      : [-\pi,   \pi]   (±\pi)
"""

from __future__ import annotations

import numpy as np

from .transforms import Transforms

class Euler:

    GIMBAL_LOCK_THRESHOLD = np.deg2rad(85.0)  # pitch 이 이상이면 경고

    def __init__(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
        """
        Args:
            roll, pitch, yaw: [rad]
        """
        self._rpy = np.array([roll, pitch, yaw], dtype=float)

    # ── 생성 클래스 메서드 ────────────────────────────────────────────────────

    @classmethod
    def zero(cls) -> "Euler":
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def from_degrees(cls, roll_deg: float, pitch_deg: float,
                     yaw_deg: float) -> "Euler":
        return cls(*np.deg2rad([roll_deg, pitch_deg, yaw_deg]))

    @classmethod
    def from_array(cls, rpy: np.ndarray) -> "Euler":
        """[roll, pitch, yaw] [rad] 배열에서 생성."""
        return cls(*rpy)

    @classmethod
    def from_quaternion(cls, q: np.ndarray) -> "Euler":
        """쿼터니언 [x,y,z,w] (scalar-last)에서 생성."""
        return cls.from_array(Transforms.quat_to_euler(q))

    @classmethod
    def from_dcm(cls, R: np.ndarray) -> "Euler":
        """DCM에서 생성."""
        return cls.from_array(Transforms.dcm_to_euler(R))

    # ── 속성 ─────────────────────────────────────────────────────────────────

    @property
    def roll(self)  -> float: return float(self._rpy[0])

    @property
    def pitch(self) -> float: return float(self._rpy[1])

    @property
    def yaw(self)   -> float: return float(self._rpy[2])

    @property
    def roll_deg(self)  -> float: return float(np.degrees(self._rpy[0]))

    @property
    def pitch_deg(self) -> float: return float(np.degrees(self._rpy[1]))

    @property
    def yaw_deg(self)   -> float: return float(np.degrees(self._rpy[2]))

    @property
    def rpy(self) -> np.ndarray:
        """[roll, pitch, yaw] [rad]."""
        return self._rpy.copy()

    @property
    def rpy_deg(self) -> np.ndarray:
        """[roll, pitch, yaw] [deg]. 플롯, 로그 출력에 편리."""
        return np.degrees(self._rpy)

    # ── 각도 정규화 ───────────────────────────────────────────────────────────

    def wrap(self) -> "Euler":
        """
        모든 각도를 [-π, π]로 정규화.
        yaw가 180° 경계를 넘을 때 시계열 플롯이 튀는 현상 방지.
        """
        return Euler.from_array(
            np.array([(a + np.pi) % (2*np.pi) - np.pi for a in self._rpy])
        )

    @staticmethod
    def unwrap_series(rpy_series: np.ndarray) -> np.ndarray:
        """
        시계열 Euler 배열의 연속성 복원.
        rpy_series: shape (N, 3) [rad]

        np.unwrap은 1D만 처리하므로 축별로 적용.
        yaw가 ±180° 경계에서 불연속 점프하는 플롯을 부드럽게 만듦.
        """
        result = rpy_series.copy()
        for i in range(3):
            result[:, i] = np.unwrap(rpy_series[:, i])
        return result

    # ── 짐벌락 감지 ───────────────────────────────────────────────────────────

    def is_near_gimbal_lock(self) -> bool:
        """
        pitch가 ±85° 이상이면 짐벌락 근처로 판단.
        이 상태에서 Euler 기반 연산 결과는 신뢰 불가.
        """
        return abs(self._rpy[1]) >= self.GIMBAL_LOCK_THRESHOLD

    # ── 변환 ──────────────────────────────────────────────────────────────────

    def to_quaternion(self) -> np.ndarray:
        """→ 쿼터니언 [x,y,z,w] (scalar-last)."""
        return Transforms.euler_to_quat(*self._rpy)

    def to_dcm(self) -> np.ndarray:
        """→ DCM 3×3."""
        return Transforms.euler_to_dcm(*self._rpy)

    # ── 오차 계산 ─────────────────────────────────────────────────────────────

    def error_from(self, other: "Euler") -> "Euler":
        """
        단순 차이 [rad]. 각 축 독립 계산.

        주의: 짐벌락 근처에서 roll/yaw 오차가 의미 없어짐.
              정밀한 오차는 geodesic_error (Quaternion) 사용 권장.
        """
        diff = self._rpy - other._rpy
        # [-π, π] 범위로 정규화
        diff = (diff + np.pi) % (2*np.pi) - np.pi
        return Euler.from_array(diff)

    # ── 출력 ──────────────────────────────────────────────────────────────────

    def __repr__(self) -> str:
        r, p, y = self.rpy_deg
        lock = " [NEAR GIMBAL LOCK]" if self.is_near_gimbal_lock() else ""
        return f"Euler(roll={r:.2f}°, pitch={p:.2f}°, yaw={y:.2f}°){lock}"
