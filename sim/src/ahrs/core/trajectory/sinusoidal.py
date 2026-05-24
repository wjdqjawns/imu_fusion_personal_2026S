"""
File Name: ./src/ahrs/core/trajectory/sinusoidal.py
Author: Beomjun Chung
Updated: 2026-05-24

Description:
    SinusoidalTrajectory — 축별 sin 함수 궤적

    단일 주파수:
        roll(t)  = A_r · sin(2π·f_r·t)
        pitch(t) = A_p · sin(2π·f_p·t + φ_p)
        yaw(t)   = A_y · sin(2π·f_y·t + φ_y)

    주파수 스윕 (frequencies_hz 리스트):
        각 주파수 구간을 이어붙여 하나의 연속 궤적으로 생성.
        구간 경계에서 위상 연속성 보장.

    Config 예시:
        trajectory:
          type: sinusoidal
          sinusoidal:
            frequencies_hz: [0.1, 0.5, 1.0, 2.0, 5.0]
            amplitude_deg: 30.0
            axes: [roll, pitch, yaw]
            phase_offset_deg: [0, 45, 90]
            duration_per_freq_s: null   # null → duration / n_freqs
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from ahrs.core.trajectory.base import BaseTrajectory, TruthData
from ahrs.utils.transforms import Transforms


class SinusoidalTrajectory(BaseTrajectory):
    """
    sin 궤적. 단일 주파수 또는 여러 주파수 스윕.

    Args:
        frequencies_hz:      주파수 목록 [Hz]
        amplitude_deg:       진폭 [deg]
        axes:                활성화 축 subset ('roll', 'pitch', 'yaw')
        phase_offset_deg:    축별 위상 오프셋 [deg]  (roll, pitch, yaw 순)
        duration_per_freq_s: 주파수당 구간 길이 [s]. None이면 총 duration/n_freqs.
    """

    def __init__(
        self,
        frequencies_hz: Sequence[float] = (1.0,),
        amplitude_deg: float = 30.0,
        axes: Sequence[str] = ("roll", "pitch", "yaw"),
        phase_offset_deg: Sequence[float] = (0.0, 0.0, 0.0),
        duration_per_freq_s: float | None = None,
    ):
        self._freqs    = list(frequencies_hz)
        self._amp      = np.deg2rad(float(amplitude_deg))
        self._axes_act = [a.lower() for a in axes]
        self._phase    = np.deg2rad(np.array(list(phase_offset_deg), dtype=float))
        if len(self._phase) < 3:
            pad = np.zeros(3 - len(self._phase))
            self._phase = np.concatenate([self._phase, pad])
        self._dur_per  = duration_per_freq_s

    def generate(self, dt: float, duration: float,
                 initial_q: np.ndarray | None = None) -> TruthData:
        n_freqs = len(self._freqs)
        dur_each = (self._dur_per if self._dur_per is not None
                    else duration / n_freqs)

        segments: list[TruthData] = []
        t_offset = 0.0
        # phase continuity: track ending angle per axis across segments
        last_angle = np.zeros(3)  # [roll, pitch, yaw] at end of prev segment

        for freq in self._freqs:
            seg = self._generate_segment(dt, dur_each, freq, t_offset, last_angle)
            segments.append(seg)
            t_offset += dur_each
            # last angles for phase continuity (end of this segment)
            last_angle = np.array([Transforms.quat_to_euler(seg.q[-1])])
            last_angle = last_angle.flatten()

        return self._concat(segments)

    # ── private ───────────────────────────────────────────────────────────────
    def _generate_segment(self, dt: float, duration: float,
                          freq: float, t_offset: float,
                          start_angle_rad: np.ndarray) -> TruthData:
        t_local = np.arange(0.0, duration, dt)
        n = len(t_local)

        omega_f = 2.0 * np.pi * freq

        # axis activation mask
        mask = np.array([
            1.0 if "roll"  in self._axes_act else 0.0,
            1.0 if "pitch" in self._axes_act else 0.0,
            1.0 if "yaw"   in self._axes_act else 0.0,
        ])

        q_arr     = np.zeros((n, 4))
        omega_arr = np.zeros((n, 3))

        for k, t_k in enumerate(t_local):
            # sin angle: starts from 0 at t=0 of each segment
            angles = self._amp * mask * np.sin(omega_f * t_k + self._phase)
            # angular velocity (analytical derivative)
            rates  = self._amp * mask * omega_f * np.cos(omega_f * t_k + self._phase)

            roll, pitch, yaw = angles
            q = Transforms.euler_to_quat(roll, pitch, yaw)
            q_arr[k] = q

            # omega in body frame: W^{-1}(angles) · rates
            omega_arr[k] = self._euler_rate_to_body(roll, pitch, rates)

        t_global = t_local + t_offset
        n_pos = np.zeros((n, 3))
        return TruthData(t=t_global, q=q_arr, omega=omega_arr,
                         pos=n_pos, vel=n_pos.copy(), accel_world=n_pos.copy())

    @staticmethod
    def _euler_rate_to_body(roll: float, pitch: float,
                             euler_rates: np.ndarray) -> np.ndarray:
        """
        Euler rate [φ̇, θ̇, ψ̇] → body angular velocity [p, q, r].
        W_inv (body ← euler rate):
          p = φ̇ - ψ̇·sinθ
          q = θ̇·cosφ + ψ̇·cosθ·sinφ
          r = -θ̇·sinφ + ψ̇·cosθ·cosφ
        """
        phi_d, theta_d, psi_d = euler_rates
        sin_phi, cos_phi = np.sin(roll), np.cos(roll)
        sin_th,  cos_th  = np.sin(pitch), np.cos(pitch)

        p = phi_d             - psi_d * sin_th
        q_b = theta_d * cos_phi + psi_d * cos_th * sin_phi
        r = -theta_d * sin_phi  + psi_d * cos_th * cos_phi
        return np.array([p, q_b, r])

    @staticmethod
    def _concat(segments: list[TruthData]) -> TruthData:
        t     = np.concatenate([s.t          for s in segments])
        q     = np.concatenate([s.q          for s in segments])
        omega = np.concatenate([s.omega      for s in segments])
        pos   = np.concatenate([s.pos        for s in segments])
        vel   = np.concatenate([s.vel        for s in segments])
        accel = np.concatenate([s.accel_world for s in segments])
        return TruthData(t=t, q=q, omega=omega, pos=pos, vel=vel, accel_world=accel)

    @classmethod
    def from_config(cls, cfg: dict) -> "SinusoidalTrajectory":
        return cls(
            frequencies_hz      = cfg.get("frequencies_hz", [1.0]),
            amplitude_deg       = float(cfg.get("amplitude_deg", 30.0)),
            axes                = cfg.get("axes", ["roll", "pitch", "yaw"]),
            phase_offset_deg    = cfg.get("phase_offset_deg", [0.0, 45.0, 90.0]),
            duration_per_freq_s = cfg.get("duration_per_freq_s", None),
        )
