"""
File Name: ./src/ahrs/core/calibration/mag_cal.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  자기계 hard/soft iron 캘리브레이션 — 타원체 피팅

    purpose:
        왜곡된 자기장 측정값을 타원체에 피팅하여
        hard iron(오프셋)과 soft iron(스케일/회전)을 추정.

    Notes:
        ellipsoid_fit (기본):
            scipy.optimize.least_squares로 타원체 파라미터 추정
            출력: hard_iron [uT], soft_iron_matrix [3×3]

        sphere_fit (단순):
            hard iron만 추정 (중심 이동), scale = 1
            빠르지만 soft iron 왜곡이 있으면 부정확

        보정 공식:
            m_cal = W_inv · (m_raw - h)
            여기서 W = soft iron matrix, h = hard iron vector
"""

from __future__ import annotations

import numpy as np
from typing import Literal


class MagCalibrator:
    """자기계 타원체 피팅 캘리브레이션."""

    def __init__(self, method: Literal["ellipsoid_fit", "sphere_fit"] = "ellipsoid_fit"):
        self._method      = method
        self._hard_iron   = np.zeros(3, dtype=float)
        self._soft_iron_W = np.eye(3, dtype=float)    # 보정 행렬 W_inv
        self._scale       = 1.0

    def fit(self, mag_data: np.ndarray) -> None:
        """
        Args:
            mag_data: 다양한 방향의 자기장 측정값 [uT], shape (N, 3)
        """
        if self._method == "sphere_fit":
            self._fit_sphere(mag_data)
        else:
            self._fit_ellipsoid(mag_data)

    def apply(self, mag_raw: np.ndarray) -> np.ndarray:
        """
        추정된 파라미터로 측정값 보정.
        m_cal = W_inv · (m_raw - hard_iron)
        """
        return self._soft_iron_W @ (mag_raw - self._hard_iron)

    @property
    def hard_iron(self) -> np.ndarray:
        return self._hard_iron.copy()

    @property
    def soft_iron_inv(self) -> np.ndarray:
        return self._soft_iron_W.copy()

    def report(self) -> dict:
        return {
            "hard_iron_uT":   self._hard_iron.tolist(),
            "soft_iron_inv":  self._soft_iron_W.tolist(),
        }

    # ── private ───────────────────────────────────────────────────────────────

    def _fit_sphere(self, data: np.ndarray) -> None:
        """구 피팅: 중심 = hard iron, 반지름 = 평균 크기."""
        self._hard_iron = np.mean(data, axis=0)
        self._soft_iron_W = np.eye(3)

    def _fit_ellipsoid(self, data: np.ndarray) -> None:
        """
        타원체 피팅 (least-squares).
        Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
        """
        try:
            from scipy.optimize import least_squares as _ls
            hard, soft_inv = self._algebraic_ellipsoid_fit(data)
            self._hard_iron   = hard
            self._soft_iron_W = soft_inv
        except Exception:
            # fallback to sphere fit
            self._fit_sphere(data)

    @staticmethod
    def _algebraic_ellipsoid_fit(data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        대수적 타원체 피팅.
        일반 이차 형식: x^T A x + b^T x + c = 0
        A는 3×3 대칭 행렬, 타원체 축 방향과 크기를 인코딩.
        """
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        n = len(x)

        # 설계 행렬 (10개 파라미터: Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + const)
        D = np.column_stack([
            x**2, y**2, z**2,
            2*x*y, 2*x*z, 2*y*z,
            2*x, 2*y, 2*z,
            np.ones(n),
        ])

        # 최소 norm 해 (SVD)
        _, _, Vt = np.linalg.svd(D, full_matrices=False)
        v = Vt[-1]  # 마지막 우특이벡터 = 최소 특이값 방향

        A_mat = np.array([
            [v[0], v[3], v[4]],
            [v[3], v[1], v[5]],
            [v[4], v[5], v[2]],
        ])
        b_vec = np.array([v[6], v[7], v[8]])
        c_sc  = v[9]

        # hard iron: center = -A^-1 · b / 2
        try:
            A_inv = np.linalg.inv(A_mat)
        except np.linalg.LinAlgError:
            return np.zeros(3), np.eye(3)

        hard_iron = -0.5 * A_inv @ b_vec

        # soft iron: A_inv 기반 정규화 행렬
        # W_corr = A^(1/2) (sphere로 되돌리는 변환)
        eigvals, eigvecs = np.linalg.eigh(A_mat)
        if np.any(eigvals <= 0):
            return hard_iron, np.eye(3)

        # soft iron 보정 행렬: W_inv = scale · (A_mat)^(1/2)
        # 구에 가깝게 정규화: 반지름 = 1 기준
        A_sqrt = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
        mean_radius = np.mean(np.linalg.norm(data - hard_iron, axis=1))
        soft_inv = A_sqrt / (np.trace(A_sqrt) / 3.0) if mean_radius > 0 else np.eye(3)

        return hard_iron, soft_inv

    @classmethod
    def from_config(cls, cfg: dict) -> "MagCalibrator":
        method = cfg.get("method", "ellipsoid_fit")
        return cls(method=method)
