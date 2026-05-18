"""
File Name: ./src/ahrs/core/calibration/accel_cal.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  가속도계 6-position 캘리브레이션

    purpose:
        6 방향(±x, ±y, ±z) 정지 자세에서 bias + scale factor 추정.
        각 방향에서 중력 ±g가 해당 축에만 작용한다는 가정 사용.

    Notes:
        - 각 position: hold_duration_s 동안 평균값 사용
        - 출력: bias [m/s²], scale [dimensionless]
        - 간단한 least-squares 추정
"""

from __future__ import annotations

import numpy as np


class AccelCalibrator:
    """6-position 가속도계 캘리브레이션."""

    def __init__(self, gravity_m_s2: float = 9.80665):
        self._g     = float(gravity_m_s2)
        self._bias  = np.zeros(3, dtype=float)
        self._scale = np.ones(3, dtype=float)

    def estimate(self, measurements: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
        """
        Args:
            measurements: {position_label: mean_accel [m/s²]}
                          position_label in {"+x","-x","+y","-y","+z","-z"}

        Returns:
            bias [m/s²], scale [dimensionless]
        """
        pairs = [("+x", "-x", 0), ("+y", "-y", 1), ("+z", "-z", 2)]
        bias  = np.zeros(3)
        scale = np.ones(3)

        for pos, neg, i in pairs:
            if pos in measurements and neg in measurements:
                p = measurements[pos][i]
                n = measurements[neg][i]
                bias[i]  = (p + n) / 2.0
                scale[i] = (p - n) / (2.0 * self._g)

        self._bias  = bias
        self._scale = scale
        return self._bias.copy(), self._scale.copy()

    def apply(self, accel_raw: np.ndarray) -> np.ndarray:
        """추정된 파라미터로 원시 측정값 보정."""
        return (accel_raw - self._bias) / (self._scale + 1e-12)

    def report(self) -> dict:
        return {
            "bias_m_s2": self._bias.tolist(),
            "bias_g":    (self._bias / 9.80665).tolist(),
            "scale":     self._scale.tolist(),
        }
