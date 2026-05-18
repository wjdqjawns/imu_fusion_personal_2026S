from __future__ import annotations

import numpy as np

def wrap_angle(angle: float, min_val: float = -np.pi, max_val: float = np.pi) -> float:
    range_size = max_val - min_val
    wrapped_angle = (angle - min_val) % range_size + min_val
    
    return wrapped_angle