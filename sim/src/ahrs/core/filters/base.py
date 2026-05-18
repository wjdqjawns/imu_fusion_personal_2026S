"""
File Name: ./src/ahrs/core/filters/base.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  AhrsFilter ABC — 모든 AHRS 필터의 공통 인터페이스

    purpose:
        runner.py가 필터 구현에 독립적으로 동일한 인터페이스로 구동할 수 있도록
        update()와 reset() 메서드를 강제.

    Notes:
        - quaternion 컨벤션: scalar-last [x, y, z, w]
        - update()는 (q, bias) 튜플 반환
        - 모든 입력 단위: SI (rad/s, m/s², uT)
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import numpy as np

class AhrsFilter(ABC):
    """AHRS 필터 추상 기반 클래스."""

    name: str = "base"

    @abstractmethod
    def update(self, gyro: np.ndarray, accel: np.ndarray,
               mag: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """
        한 스텝 업데이트.

        Args:
            gyro:  [rad/s],  shape (3,)
            accel: [m/s²],   shape (3,)
            mag:   [uT],     shape (3,)
            dt:    [s]

        Returns:
            q:    [x,y,z,w], shape (4,)
            bias: [rad/s],   shape (3,)  (추정 gyro bias)
        """
        ...

    def reset(self) -> None:
        """필터 상태 초기화. 필요 시 서브클래스에서 오버라이드."""
        pass

    @property
    def quaternion(self) -> np.ndarray:
        """현재 자세 쿼터니언 [x,y,z,w]."""
        raise NotImplementedError