"""
File Name: ./src/ahrs/pipeline/runner.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Multistage AHRS Simulation Pipeline Runner.

    Stage 1 — Config load
    Stage 2 — Export setup  (dirs / logger file handler / plotter)
    Stage 3 — Environment   (gravity, magnetic field, thermal)
    Stage 4 — Sensor setup  (IMUSensor with noise model)
    Stage 5 — Noise characterization (Allan / PSD / FFT / Mag cal)
    Stage 6 — State estimation  (GT → sensor → filter loop → eval)
    Stage 7 — Report
"""

from __future__ import annotations

from pathlib import Path
from datetime import datetime

import numpy as np

from ahrs.core.env.environment import Environment
from ahrs.core.model.sensor import IMUSensor
from ahrs.utils.configger import load_config
from ahrs.utils.exporter import build_export
from ahrs.utils.logger import get_logger, setup_logging
from ahrs.utils.plotter import Plotter
# from ahrs.utils.reporter import FilterReport
from ahrs.pipeline.stage.stage_characterization import run_characterization
from ahrs.pipeline.stage.stage_estimation import run_estimation
from ahrs.pipeline.stage.stage_report import run_report

logger = get_logger(__name__)

def run(config_path: str | Path, dry_run: bool = False, log_level: str = "INFO") -> None:
    setup_logging(level=log_level, console=True)
    start_time = datetime.now()
    logger.info("============================================================")
    logger.info("Starting AHRS simulation pipeline at %s", start_time.strftime("%Y-%m-%d %H:%M:%S"))
    logger.info("============================================================")

    # Stage 1: Config
    logger.info("=== Stage 1: Config ===")
    logger.info("[STAGE 1] config path: %s", config_path)
    cfg      = load_config(config_path)
    sim_cfg  = cfg["simulation"]
    dt       = float(sim_cfg["dt"])
    duration = float(sim_cfg["duration"])
    seed     = int(sim_cfg.get("random_seed", 42))
    logger.info("[STAGE 1] dt=%.4f [sec]  duration=%.1f [sec]  seed=%d", dt, duration, seed)
    if dry_run:
        logger.info("[STAGE 1] Dry run — config OK.")
        return
    logger.info("[STAGE 1] Config loaded complete")

    # Stage 2: Export (dirs + file logger + plotter)
    logger.info("=== Stage 2: Export ===")
    ctx = build_export(cfg)
    log_cfg = cfg.get("export", {}).get("log", {})
    setup_logging(log_dir = ctx.log_dir, level = log_cfg.get("level", log_level), console = log_cfg.get("console", True), fmt = log_cfg.get("format", "log"))
    plotter = Plotter.from_config(cfg)
    logger.info("[STAGE 2] Output | %s", ctx.export_dir)
    logger.info("[STAGE 2] Export setup complete")

    # Stage 3: Environment
    logger.info("=== Stage 3: Environment ===")
    env = Environment.from_config(cfg["env"])
    logger.info("[STAGE 3] Gravity Env  [m/s^2] | %s", np.round(env.gravity_ned, 2).tolist())
    logger.info("[STAGE 3] Magnetic Env    [uT] | %s", np.round(env.mag_ned, 2).tolist())
    logger.info("[STAGE 3] Temperature Env  [K] | %s", env.temp_magnitude)
    logger.info("[STAGE 3] Environment setup complete")

    # Stage 4: Sensor
    logger.info("=== Stage 4: Sensor ===")
    sensor = IMUSensor.from_config(cfg, env, seed=seed)
    logger.info("[STAGE 4] Sensor setup complete")

    # Stage 5: Noise characterization
    logger.info("=== Stage 5: Noise Characterization ===")
    char_results = run_characterization(cfg, env, sensor, dt, ctx, plotter)
    logger.info("[STAGE 5] Noise characterization complete. Summary saved to %s", ctx.report_dir / "calibration_summary.json")

    # Stage 6: State estimation
    logger.info("=== Stage 6: State Estimation ===")
    # est_results = run_estimation(cfg, env, sensor, dt, duration, ctx, plotter)
    logger.info("[STAGE 6] Estimation complete")

    # Stage 7: Report
    logger.info("=== Stage 7: Report ===")
    # run_report(cfg, char_results, est_results, ctx, plotter)
    logger.info("[STAGE 7] Report saved | %s", ctx.report_dir)

    # Final log + return
    end_time = datetime.now()
    logger.info("============================================================")
    logger.info("Simulation Complete | Duration: %f seconds", (end_time - start_time).total_seconds())
    logger.info("============================================================")

    return