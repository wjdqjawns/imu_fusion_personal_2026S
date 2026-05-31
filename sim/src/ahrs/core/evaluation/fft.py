"""
File Name: ./src/ahrs/core/evaluation/fft.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:

"""

from __future__ import annotations

import numpy as np

class FFTAnalysis:
    def __init__(self, data: np.ndarray, dt: float, n_tau: int = 100):
        