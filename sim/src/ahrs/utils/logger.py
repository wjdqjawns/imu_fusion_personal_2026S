#*************************************************************************/
# File Name: ./src/utils/logger.py
# Author: Beomjun Chung
# Updated: 2026-03-22
#
# Description:
#   Logger setup for ahrs sim runtime.
#*************************************************************************/

from __future__ import annotations

import logging
from logging.handlers import MemoryHandler
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT_LOGGER_NAME = "ahrs_sim"
LOGGER_FORMAT = "[%(levelname)s] %(message)s"

_LEVEL_MAP: dict[str, int] = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}

def _resolve_log_file_path(
    logger_name: str,
    config: dict[str, Any],
    start_time: datetime | None = None,
) -> Path | None:
    if start_time is None:
        return None

    active_sim = config["active_sim"]
    common_cfg = config["common"]
    simulation_cfg = config["simulation"][active_sim]
    
    export_dir = simulation_cfg["export_dir"]
    export_root = export_dir["root"]
    export_folders = export_dir["folders"]

    log_dir = Path(export_root) / "log" if "log" in export_folders else export_root

    log_pattern = common_cfg["sim_naming"]["output_naming"]["log_pattern"]
    if not isinstance(log_pattern, str) or not log_pattern.strip():
        return None

    file_name = log_pattern.format(timestamp=start_time.strftime("%Y%m%d_%H%M%S"), sim_name=active_sim, logger_name=logger_name)

    log_file = log_dir / file_name

    return log_file if log_file.suffix in (".log", ".txt") else None

def get_logger(logger_name: str, level: str = "INFO") -> logging.Logger:
    numeric_level = _LEVEL_MAP.get(level.upper(), logging.INFO)

    sub_logger = logging.getLogger(f"{ROOT_LOGGER_NAME}.{logger_name}")
    sub_logger.setLevel(numeric_level)

    return sub_logger

def setup_logging(
    level: str = "INFO",
    config: dict[str, Any] | None = None,
    start_time: datetime | None = None,
) -> logging.Logger:
    numeric_level = _LEVEL_MAP.get(level.upper(), logging.INFO)

    logger = logging.getLogger(ROOT_LOGGER_NAME)
    logger.setLevel(numeric_level)

    if not logger.handlers:
        stream_formatter = logging.Formatter(LOGGER_FORMAT)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(numeric_level)
        stream_handler.setFormatter(stream_formatter)
        logger.addHandler(stream_handler)

        buffer_handler = MemoryHandler(capacity=10000, target=None)
        buffer_handler.setLevel(numeric_level)
        logger.addHandler(buffer_handler)

        logger.propagate = False

    if config is not None and not any(isinstance(h, logging.FileHandler) for h in logger.handlers):
        log_file = _resolve_log_file_path(
            logger_name=ROOT_LOGGER_NAME,
            config=config,
            start_time=start_time,
        )
        if log_file is not None:
            log_file.parent.mkdir(parents=True, exist_ok=True)
            file_formatter = logging.Formatter(
                fmt="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            )
            file_handler = logging.FileHandler(log_file, encoding="utf-8")
            file_handler.setLevel(numeric_level)
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)

            for handler in list(logger.handlers):
                if isinstance(handler, MemoryHandler):
                    handler.target = file_handler
                    handler.flush()
                    logger.removeHandler(handler)
                    handler.close()
            logger.info("File logging enabled. Log file: %s", log_file)
        else:
            logger.warning("Log file path could not be resolved. File logging disabled.")

    logger.info("Logger initialized with log file: %s", log_file)
    return logger