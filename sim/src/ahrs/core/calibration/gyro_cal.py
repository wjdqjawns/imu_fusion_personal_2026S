"""
File Name: ./src/ahrs/core/calibration/gyro_cal.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  자이로스코프 정적 바이어스 추정

    purpose:
        정지 상태에서 자이로 출력의 평균값 = turn-on bias 추정.
        필터 초기화 시 측정된 바이어스로 초기 상태 설정.

    Notes:
        - 입력: 정지 구간 자이로 시계열 [rad/s], shape (N, 3)
        - 출력: 추정 바이어스 [rad/s], shape (3,)
        - 단순 평균. 더 정밀한 추정은 Allan variance 기반 BI 추정 사용.
"""

from __future__ import annotations

import numpy as np


class GyroBiasCalibrator:
    """정지 구간 자이로 바이어스 추정."""

    def __init__(self):
        self._bias: np.ndarray | None = None
        self._std:  np.ndarray | None = None

    def estimate(self, gyro_static: np.ndarray) -> np.ndarray:
        """
        Args:
            gyro_static: 정지 구간 자이로 측정값 [rad/s], shape (N, 3)

        Returns:
            bias_rad_s: 추정 바이어스 [rad/s], shape (3,)
        """
        self._bias = np.mean(gyro_static, axis=0)
        self._std  = np.std(gyro_static, axis=0)
        return self._bias.copy()

    @property
    def bias(self) -> np.ndarray | None:
        return self._bias.copy() if self._bias is not None else None

    @property
    def std(self) -> np.ndarray | None:
        return self._std.copy() if self._std is not None else None

    def report(self) -> dict:
        if self._bias is None:
            return {}
        return {
            "bias_deg_s": np.rad2deg(self._bias).tolist(),
            "std_deg_s":  np.rad2deg(self._std).tolist(),
        }
