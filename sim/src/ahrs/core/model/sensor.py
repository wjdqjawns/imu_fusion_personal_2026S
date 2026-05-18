"""
File Name: ./src/core/model/sensor.py
Author: Beomjun Chung
Updated: 2026-05-17

Description:
    sensor.py — IMU 센서 통합 모델

    역할:
        trajectory에서 받은 ground truth 값을 세 센서 모델에 넣어
        노이즈가 섞인 측정값을 만들어냄.
        runner.py가 매 스텝마다 호출하는 핵심 인터페이스.

    운동학 → 측정값 변환 흐름:
        ┌─ Trajectory ──────────────────────────────────────────┐
        │  q_true    : real orientation (quaternion)            │
        │  omega_true: real angular velocity [rad/s] (body f.)  │
        │  pos_true  : real position [m]                        │
        │  vel_true  : real velocity [m/s] (world f.)           │
        └───────────────────────────────────────────────────────┘
                │
                ▼
        ┌─ IMUSensor.measure() ─────────────────────────────────┐
        │                                                       │
        │  Gyroscope.measure(omega_true, accel_body, dt)        │
        │    → omega_meas [rad/s]                               │
        │                                                       │
        │  Accelerometer.measure(specific_force_body, dt)       │
        │    specific_force = R^T·g - a_linear                  │
        │    → accel_meas [m/s²]                                │
        │                                                       │
        │  Magnetometer.measure(mag_body, dt)                   │
        │    mag_body = R^T · m_world(coordinate trans.)        │
        │    → mag_meas [uT]                                    │
        │                                                       │
        └───────────────────────────────────────────────────────┘
                │
                ▼
        ┌─ Filters ─────────────────────────────────────────────┐
        │  omega_meas, accel_meas, mag_meas input               │
        │  → q_est (prediction attitude)                        │
        └───────────────────────────────────────────────────────┘

    비력(Specific Force)이란:
        가속도계가 측정하는 것은 중력을 포함한 비력(specific force).
        a_sf = a_total - g  (뉴턴 관성계)
        body frame으로 변환: a_sf_body = R^T · (a_world - g_world)
        정지 시: a_world = 0  →  a_sf_body = -R^T · g_world = [0, 0, g] (NED)
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from ahrs.core.model.gyroscope import Gyroscope
from ahrs.core.model.accelerometer import Accelerometer
from ahrs.core.model.magnetometer import Magnetometer
from ahrs.core.orientation.transforms import Transforms

@dataclass
class IMUMeasurement:
    t:       float        # [s]
    dt:      float        # [s]

    gyro:    np.ndarray   # [rad/s]
    accel:   np.ndarray   # [m/s²]
    mag:     np.ndarray   # [uT]

    is_dynamic:   bool = False
    is_mag_disturbed: bool = False

@dataclass
class GroundTruth:
    q:       np.ndarray   # attitude quaternion [x,y,z,w] scalar-last
    omega:   np.ndarray   # angular velocity [rad/s], body frame
    pos:     np.ndarray   # position [m], world frame
    vel:     np.ndarray   # velocity [m/s], world frame
    accel_world: np.ndarray  # linear acceleration [m/s²], world frame (gravity excluded)
    mag_world:   np.ndarray  # magnetic field [uT], world frame

class IMUSensor:
    def __init__(
        self,
        gyro: Gyroscope, # gyro model
        accel: Accelerometer, # accelerometer model
        mag: Magnetometer, # magnetometer model
        gravity_world: np.ndarray, # gravity vector in world frame [m/s²], e.g. NED: [0, 0, 9.80665]
        mag_world: np.ndarray, # standard magnetic field in world frame [uT], e.g. NED: [north, east, down]
    ):
        self.gyro  = gyro
        self.accel = accel
        self.mag   = mag

        self.gravity_world = np.array(gravity_world)
        self.mag_world     = np.array(mag_world)
        self._ref_mag_norm = np.linalg.norm(self.mag_world)

    def measure(self, truth: GroundTruth, t: float, dt: float) -> IMUMeasurement:
        R = Transforms.quat_to_dcm(truth.q)   # body2world transformation
        R_T = R.T                  # world2body transformation

        # 비력 (specific force): 가속도계가 측정하는 것
        # = -중력 + 선형 가속도, body frame
        specific_force_body = R_T @ (-self.gravity_world + truth.accel_world)

        # 자기장 body frame 변환
        mag_body = R_T @ self.mag_world

        # ── 각 센서 측정 ──────────────────────────────────────────────────
        gyro_meas  = self.gyro.measure(truth.omega, specific_force_body, dt)
        accel_meas = self.accel.measure(specific_force_body, dt)
        mag_meas   = self.mag.measure(mag_body, dt)

        # ── 외란 감지 ─────────────────────────────────────────────────────
        is_dyn = self.accel.is_dynamic(accel_meas, np.linalg.norm(self.gravity_world))
        is_mag = self.mag.is_disturbed(mag_meas, self._ref_mag_norm)

        return IMUMeasurement(
            gyro=gyro_meas,
            accel=accel_meas,
            mag=mag_meas,
            t=t,
            dt=dt,
            is_dynamic=is_dyn,
            is_mag_disturbed=is_mag,
        )

    def set_disturbance(self, mag_disturbance_uT: np.ndarray | None = None) -> None:
        """외부 자기 외란 주입. runner에서 이벤트 타임에 호출."""
        if mag_disturbance_uT is not None:
            self.mag.set_disturbance(mag_disturbance_uT)
        else:
            self.mag.clear_disturbance()

    def reset(self) -> None:
        """시뮬 재시작. 모든 센서 상태 초기화."""
        self.gyro.reset()
        self.accel.reset()
        self.mag.reset()

    # ── 생성 헬퍼 ─────────────────────────────────────────────────────────────

    @classmethod
    def from_config(cls, cfg: dict, seed: int | None = None) -> "IMUSensor":
        """
        config.yaml 전체에서 IMUSensor 생성.

        Expected structure:
            imu:
              gyroscope: {...}
              accelerometer: {...}
              magnetometer: {...}
            earth:
              gravity_m_s2: 9.7957
              magnetic_field_uT:
                north: 20.0
                east:  0.0
                down:  43.0
        """
        rng = np.random.default_rng(seed)
        seeds = rng.integers(0, 2**31, size=3)

        from ahrs.core.env.environment import Environment

        gyro  = Gyroscope.from_config(cfg["imu"]["gyroscope"],     int(seeds[0]))
        accel = Accelerometer.from_config(cfg["imu"]["accelerometer"], int(seeds[1]))
        mag   = Magnetometer.from_config(cfg["imu"]["magnetometer"],   int(seeds[2]))

        env = Environment.from_config(cfg["env"])
        gravity_world = env.gravity_ned
        mag_world     = env.mag_ned

        return cls(
            gyro=gyro,
            accel=accel,
            mag=mag,
            gravity_world=gravity_world,
            mag_world=mag_world,
        )