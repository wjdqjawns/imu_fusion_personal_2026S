"""
File Name: ./src/ahrs/core/model/sensor.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    IMUSensor — Trajectory(GT) → Sensor Measurement 변환 인터페이스

    두 단계로 동작:
        1. 측정 방정식 (물리): 세계 좌표 GT → body frame 이상적 측정값
               specific_force_body = R^T · (-g_world + a_world)
               mag_body            = R^T · m_world
               omega_body          = omega_true  (자이로는 body frame 그대로)

        2. 센서 모델 (노이즈): 각 센서 클래스가 이상적 측정값 + noise 적용
               Gyroscope   : omega_meas  = omega_body  + scale·bias·BI·RRW·ARW·g-sens
               Accelerometer: accel_meas  = sf_body     + scale·bias·BI·VRW
               Magnetometer : mag_meas    = hard/soft·mag_body + BI·white_noise

    외란(disturbance):
        runner가 측정값에 직접 더하는 방식으로 처리.
        센서 모델은 외란을 알지 못함 — GT와 실제 측정 사이의 gap만 담당.

    GroundTruth:
        trajectory가 생성한 진짜 상태. IMUSensor.measure()의 입력 스펙.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ahrs.core.model.gyroscope import Gyroscope
from ahrs.core.model.accelerometer import Accelerometer
from ahrs.core.model.magnetometer import Magnetometer
from ahrs.utils.logger import get_logger
from ahrs.utils.transforms import Transforms

logger = get_logger(__name__)

@dataclass
class IMUMeasurement:
    """sensor.measure() 출력 — 노이즈가 포함된 raw 측정값."""
    t:     float         # timestamp [s]
    dt:    float         # sample interval [s]
    gyro:  np.ndarray   # angular velocity [rad/s], body frame
    accel: np.ndarray   # specific force [m/s²], body frame
    mag:   np.ndarray   # magnetic field [µT], body frame

@dataclass
class GroundTruth:
    """trajectory가 생성하는 진짜 상태. IMUSensor.measure()의 입력."""
    q:           np.ndarray   # attitude quaternion [x, y, z, w] scalar-last
    omega:       np.ndarray   # angular velocity [rad/s], body frame
    pos:         np.ndarray   # position [m], world frame
    vel:         np.ndarray   # velocity [m/s], world frame
    accel_world: np.ndarray   # linear acceleration [m/s²], world frame (gravity excluded)
    mag_world:   np.ndarray   # magnetic field [µT], world frame

class IMUSensor:
    """
    3-axis IMU 시뮬레이션 인터페이스.

    Args:
        gyro:           Gyroscope 센서 모델
        accel:          Accelerometer 센서 모델
        mag:            Magnetometer 센서 모델
        gravity_world:  중력 벡터, world frame [m/s²]  (NED: [0, 0, +g])
        mag_world:      지자기 기준 벡터, world frame [µT]
    """
    def __init__(
        self,
        gyro:          Gyroscope,
        accel:         Accelerometer,
        mag:           Magnetometer,
        gravity_world: np.ndarray, # [m/s²]; NED Frame
        mag_world:     np.ndarray, # [uT]; NED Frame
        temp_world: float = 298.15, # [K]
    ):
        self.gyro  = gyro
        self.accel = accel
        self.mag   = mag

        self._gravity_world = np.asarray(gravity_world, dtype=float)
        self._mag_world     = np.asarray(mag_world,     dtype=float)
        self._temp_world    = temp_world

    def measure(self, truth: GroundTruth, t: float, dt: float) -> IMUMeasurement:
        """
        measure sensor with gt states + noise

        Step 1. 측정 방정식 — body frame 이상적 측정값 계산
        Step 2. 각 센서 모델 — 이상적 값 + 노이즈

        Returns:
            IMUMeasurement (gyro, accel, mag) — 외란은 runner에서 추가
        """
        R   = Transforms.quat_to_dcm(truth.q)  # body → world
        R_T = R.T                               # world → body

        # 비력(specific force): 이상적 가속도계 입력
        # = R^T · (-g_world + a_linear_world)
        # NED 정지 시: [0,0,-g] (중력이 위방향 = -D축)
        specific_force_body = R_T @ (-self._gravity_world + truth.accel_world)

        # 자기장 body frame 변환
        mag_body = R_T @ self._mag_world

        gyro_meas  = self.gyro.measure(truth.omega, specific_force_body, dt)
        accel_meas = self.accel.measure(specific_force_body, dt)
        mag_meas   = self.mag.measure(mag_body, dt)

        return IMUMeasurement(t=t, dt=dt, gyro=gyro_meas, accel=accel_meas, mag=mag_meas)

    def reset(self) -> None:
        self.gyro.reset()
        self.accel.reset()
        self.mag.reset()

    @classmethod
    def from_config(
        cls,
        cfg: dict,
        env,
        seed: int | None = None,
    ) -> "IMUSensor":
        """
        Create an IMUSensor instance from config dict and environment.

        Args:
            cfg:  config dict  (cfg["sensor"])
            env:  Environment class (gravity_ned, mag_ned, temp_magnitude)
            seed: random seed for sensor noise generation (optional, for reproducibility)
        """
        rng   = np.random.default_rng(seed)
        seeds = rng.integers(0, 2**31, size=3)

        sensor_cfg = cfg["sensor"]
        gyro  = Gyroscope.from_config(sensor_cfg["gyroscope"],       int(seeds[0]))
        accel = Accelerometer.from_config(sensor_cfg["accelerometer"], int(seeds[1]))
        mag   = Magnetometer.from_config(sensor_cfg["magnetometer"],   int(seeds[2]))

        return cls(
            gyro          = gyro,
            accel         = accel,
            mag           = mag,
            gravity_world = env.gravity_ned,
            mag_world     = env.mag_ned,
            temp_world    = env.temp_magnitude,
        )