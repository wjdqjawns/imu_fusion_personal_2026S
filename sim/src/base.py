from dataclasses import dataclass
import numpy as np


@dataclass
class AhrsState:
    q: np.ndarray
    gyro_bias: np.ndarray


class AhrsFilter:
    name = "base"

    def update(self, gyro, accel, mag, dt):
        raise NotImplementedError

    @property
    def quaternion(self):
        raise NotImplementedError
