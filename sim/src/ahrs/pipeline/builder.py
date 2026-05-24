from __future__ import annotations

from pathlib import Path
from dataclasses import dataclass
from typing import Any

from ahrs.core.trajectory.base import BaseTrajectory
from ahrs.core.trajectory.static import StaticTrajectory
from ahrs.core.trajectory.circular import CircularTrajectory
from ahrs.core.trajectory.figure8 import Figure8Trajectory
from ahrs.core.trajectory.spherical import SphericalTrajectory
from ahrs.core.trajectory.sinusoidal import SinusoidalTrajectory

from ahrs.core.estimator.base import AhrsFilter
from ahrs.core.estimator.complementary import ComplementaryFilter
from ahrs.core.estimator.mahony import MahonyFilter
from ahrs.core.estimator.madgwick import MadgwickFilter
from ahrs.core.estimator.ekf import EkfFilter
from ahrs.core.estimator.ekf_euler import EkfEulerFilter
from ahrs.core.estimator.eskf import EskfFilter

def build_trajectory(cfg: dict) -> BaseTrajectory:
    traj_type = cfg["trajectory"]["type"]
    traj_cfg  = cfg["trajectory"].get(traj_type, {})

    if traj_type == "static":
        return StaticTrajectory.from_config(traj_cfg)
    elif traj_type == "circular":
        return CircularTrajectory.from_config(traj_cfg)
    elif traj_type == "figure8":
        return Figure8Trajectory.from_config(traj_cfg)
    elif traj_type == "spherical":
        return SphericalTrajectory.from_config(traj_cfg)
    elif traj_type == "sinusoidal":
        return SinusoidalTrajectory.from_config(traj_cfg)
    else:
        raise ValueError(f"Unknown trajectory type: {traj_type!r}")

def build_estimator(cfg: dict) -> dict[str, AhrsFilter]:
    est_cfg  = cfg.get("estimator", cfg.get("estimator", {}))
    run_list = est_cfg.get("run", ["complementary", "mahony", "madgwick", "ekf"])

    estimators: dict[str, AhrsFilter] = {}

    if "complementary" in run_list:
        estimators["complementary"] = ComplementaryFilter(**est_cfg.get("complementary", {}))

    if "mahony" in run_list:
        estimators["mahony"] = MahonyFilter(**est_cfg.get("mahony", {}))

    if "madgwick" in run_list:
        estimators["madgwick"] = MadgwickFilter(**est_cfg.get("madgwick", {}))

    if "ekf_quat" in run_list:
        estimators["ekf_quat"] = EkfFilter(**est_cfg.get("ekf_quat", est_cfg.get("ekf", {})))

    if "ekf_euler" in run_list:
        estimators["ekf_euler"] = EkfEulerFilter(**est_cfg.get("ekf_euler", {}))

    if "eskf" in run_list:
        estimators["eskf"] = EskfFilter(**est_cfg.get("eskf", {}))

    return estimators