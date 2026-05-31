"""
File Name: ./src/ahrs/core/evaluation/allan.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Overlapping Allan Variance 분석 도구

    purpose:
        자이로/가속도계 정적 데이터에서 ARW, BI, RRW 파라미터를 Allan deviation
        곡선의 기울기 분석으로 추출.

    Notes:
        Overlapping Allan Variance (non-overlapping 금지):
            σ²(τ) = 1 / (2·τ²·(N - 2m)) · Σ [θ(k+2m) - 2θ(k+m) + θ(k)]²
            여기서 m = τ/dt, θ(k) = 누적 각도 (자이로 적분)

        Allan deviation 기울기 구간:
            기울기 -1/2: ARW  → τ=1s에서의 Allan deviation 값 [rad/√s]
            기울기  0  : BI   → 최솟값 [rad/s]
            기울기 +1/2: RRW  → τ=3s에서의 Allan deviation 값 [rad/s/√s]

        단위 출력:
            ARW: [deg/√h]
            BI:  [deg/h]
            RRW: [deg/h/√h]
"""

from __future__ import annotations

import numpy as np

class AllanVarianceAnalysis:
    def __init__(self, data: np.ndarray, dt: float, n_tau: int = 100):
        """
        Args:
            data:  자이로 또는 가속도계 시계열 [rad/s or m/s²], shape (N,) or (N, 3)
            dt:    샘플 간격 [s]
            n_tau: Allan variance를 계산할 tau 포인트 수
        """
        self._data  = np.asarray(data, dtype=float)
        self._dt    = float(dt)
        self._n_tau = int(n_tau)
        self._tau:   np.ndarray | None = None
        self._avar:  np.ndarray | None = None
        self._adev:  np.ndarray | None = None

    def compute(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Allan variance 계산.

        Returns:
            tau:  [s], shape (n_tau,)
            adev: Allan deviation, same unit as input/√s, shape (n_tau,) or (n_tau, 3)
        """
        data = self._data
        if data.ndim == 1:
            data = data[:, np.newaxis]

        n = len(data)
        # 누적 각도 θ(k) = Σ data(i) * dt
        theta = np.cumsum(data, axis=0) * self._dt

        max_m = n // 5  # tau_max = duration/5 권장
        m_vals = np.unique(
            np.round(np.geomspace(1, max_m, self._n_tau)).astype(int)
        )
        m_vals = m_vals[m_vals >= 1]

        n_cols = data.shape[1]
        tau_list  = []
        avar_list = []

        for m in m_vals:
            tau = m * self._dt
            # overlapping allan variance
            sums = 0.0
            count = n - 2*m
            if count <= 0:
                continue
            # Σ [θ(k+2m) - 2θ(k+m) + θ(k)]²
            diff = theta[2*m:] - 2*theta[m:n-m] + theta[:n-2*m]
            avar_tau = np.mean(diff**2, axis=0) / (2.0 * tau**2)

            tau_list.append(tau)
            avar_list.append(avar_tau)

        self._tau  = np.array(tau_list)
        self._avar = np.array(avar_list)  # shape (n_tau, 3) or (n_tau, 1)
        self._adev = np.sqrt(np.abs(self._avar))

        if self._data.ndim == 1:
            self._avar = self._avar[:, 0]
            self._adev = self._adev[:, 0]

        return self._tau, self._adev

    def extract_params(self) -> dict:
        """
        Allan deviation 곡선에서 ARW, BI, RRW 추출.

        단축 추정법:
            ARW: τ=1s에서 Allan deviation
            BI:  전체 최솟값
            RRW: τ=3s에서 Allan deviation / √3

        Returns:
            dict with keys: arw_rad_sqrts, bi_rad_s, rrw_rad_s_sqrts (per axis if 3-axis)
        """
        if self._tau is None:
            self.compute()

        tau  = self._tau
        adev = self._adev

        def _extract_1d(ad: np.ndarray) -> dict:
            # ARW: τ=1s
            idx1 = np.argmin(np.abs(tau - 1.0))
            arw = float(ad[idx1])

            # BI: min
            bi = float(np.min(ad))

            # RRW: τ=3s 근방, 기울기 +1/2
            idx3 = np.argmin(np.abs(tau - 3.0))
            rrw = float(ad[idx3]) / np.sqrt(3.0) if idx3 < len(ad) else 0.0

            return {"arw": arw, "bi": bi, "rrw": rrw}

        _RAD_PER_S_TO_DEG_PER_H = np.rad2deg(1.0) * 3600.0
        _RAD_SQRTS_TO_DEG_SQRTH = np.rad2deg(1.0) * 60.0  # √s→√h factor 60

        if adev.ndim == 1:
            p = _extract_1d(adev)
            return {
                "arw_deg_per_sqrth": p["arw"] * _RAD_SQRTS_TO_DEG_SQRTH,
                "bi_deg_h":          p["bi"]  * _RAD_PER_S_TO_DEG_PER_H,
                "rrw_deg_h_sqrth":   p["rrw"] * _RAD_PER_S_TO_DEG_PER_H * 60.0,
            }
        else:
            # 3-axis
            result = {}
            axes = ["x", "y", "z"]
            for i, ax in enumerate(axes):
                p = _extract_1d(adev[:, i])
                result[f"arw_deg_per_sqrth_{ax}"] = p["arw"] * _RAD_SQRTS_TO_DEG_SQRTH
                result[f"bi_deg_h_{ax}"]           = p["bi"]  * _RAD_PER_S_TO_DEG_PER_H
                result[f"rrw_deg_h_sqrth_{ax}"]    = p["rrw"] * _RAD_PER_S_TO_DEG_PER_H * 60.0
            return result

    @property
    def tau(self) -> np.ndarray | None:
        return self._tau

    @property
    def adev(self) -> np.ndarray | None:
        return self._adev