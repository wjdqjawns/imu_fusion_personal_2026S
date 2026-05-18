"""
File Name: ./src/ahrs/core/evaluation/metrics.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  필터 성능 평가 지표 — RMSE, geodesic error

    purpose:
        필터 추정 자세와 ground truth를 비교하여 정량적 성능 지표를 계산.

    Notes:
        - Euler RMSE: 짐벌락 영향 있음, roll/pitch/yaw 개별 분석에 사용
        - Geodesic error: 짐벌락 없는 진짜 각도 오차, 전체 자세 오차 평가에 사용
        - attitude_error: [-π, π] 범위로 wrap된 Euler 차이
"""

from __future__ import annotations

import numpy as np

from ahrs.core.orientation.transforms import Transforms


def euler_rmse(euler_est: np.ndarray, euler_true: np.ndarray) -> np.ndarray:
    """
    Euler 각도 RMSE [deg].

    Args:
        euler_est:  추정 [rad], shape (N, 3) [roll, pitch, yaw]
        euler_true: 진짜 [rad], shape (N, 3)

    Returns:
        rmse_deg: [roll_rmse, pitch_rmse, yaw_rmse] [deg]
    """
    diff = euler_est - euler_true
    # [-π, π] wrap
    diff = (diff + np.pi) % (2 * np.pi) - np.pi
    return np.rad2deg(np.sqrt(np.mean(diff**2, axis=0)))


def geodesic_rmse(q_est: np.ndarray, q_true: np.ndarray) -> float:
    """
    Geodesic(각도) 오차의 RMS [deg].

    Args:
        q_est:  추정 쿼터니언 [x,y,z,w], shape (N, 4)
        q_true: 진짜 쿼터니언 [x,y,z,w], shape (N, 4)

    Returns:
        rmse_deg: geodesic RMSE [deg]
    """
    errors = np.array([
        Transforms.geodesic_error(q_est[k], q_true[k])
        for k in range(len(q_est))
    ])
    return float(np.rad2deg(np.sqrt(np.mean(errors**2))))


def geodesic_errors(q_est: np.ndarray, q_true: np.ndarray) -> np.ndarray:
    """
    Geodesic 오차 시계열 [deg].

    Returns:
        errors_deg: shape (N,)
    """
    return np.array([
        np.rad2deg(Transforms.geodesic_error(q_est[k], q_true[k]))
        for k in range(len(q_est))
    ])


def attitude_error_euler(q_est: np.ndarray, q_true: np.ndarray) -> np.ndarray:
    """
    Euler 오차 시계열 [deg] (wrap 포함).

    Returns:
        errors_deg: shape (N, 3) [roll, pitch, yaw]
    """
    n = len(q_est)
    errors = np.zeros((n, 3))
    for k in range(n):
        e_est  = Transforms.quat_to_euler(q_est[k])
        e_true = Transforms.quat_to_euler(q_true[k])
        diff = e_est - e_true
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        errors[k] = np.rad2deg(diff)
    return errors


def convergence_time(geodesic_err_deg: np.ndarray, dt: float,
                     threshold_deg: float = 2.0) -> float | None:
    """
    오차가 threshold 이하로 처음 들어오는 시간 [s].

    Returns:
        수렴 시간 [s], 수렴 못하면 None
    """
    below = np.where(geodesic_err_deg < threshold_deg)[0]
    if len(below) == 0:
        return None
    return float(below[0] * dt)
