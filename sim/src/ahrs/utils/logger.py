"""
File Name: ./src/ahrs/utils/logger.py
Author: Beomjun Chung
Updated: 2026-05-25

Description:
    Logger setup for ahrs sim runtime.

    Usage:
        # main entry point (once):
        from ahrs.utils.logger import setup_logging
        setup_logging(log_dir=ctx.log_dir, level="INFO", console=True)

        # any other module:
        from ahrs.utils.logger import get_logger
        logger = get_logger(__name__)   # → ahrs_sim.<module>
"""

from __future__ import annotations

import logging
from pathlib import Path

ROOT_LOGGER_NAME = "ahrs_sim"

_LEVEL_MAP: dict[str, int] = {
    "DEBUG":    logging.DEBUG,
    "INFO":     logging.INFO,
    "WARNING":  logging.WARNING,
    "ERROR":    logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}

_CONSOLE_FMT = "[%(levelname)s][%(name)s] %(message)s"
_FILE_FMT    = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
_CSV_FMT     = "%(asctime)s,%(levelname)s,%(name)s,%(message)s"
_DATE_FMT    = "%Y-%m-%d %H:%M:%S"


def get_logger(name: str, level: str = "INFO") -> logging.Logger:
    """
    Return a child logger under the ahrs_sim root.

    Args:
        name:  Module name (use __name__ or a short label).
        level: Log level string. Inherits root level if not explicitly set.

    Returns:
        logging.Logger namespaced as 'ahrs_sim.<name>'.
    """
    numeric_level = _LEVEL_MAP.get(level.upper(), logging.INFO)
    child = logging.getLogger(f"{ROOT_LOGGER_NAME}.{name}")
    child.setLevel(numeric_level)
    return child


def setup_logging(
    log_dir: Path | str | None = None,
    level: str = "INFO",
    console: bool = True,
    fmt: str = "log",
) -> logging.Logger:
    """
    Configure the ahrs_sim root logger. Call once at program startup.

    All child loggers (get_logger / logging.getLogger("ahrs_sim.*"))
    propagate to this root and share its handlers automatically.

    Args:
        log_dir: Directory for the log file. None → file logging disabled.
        level:   Log level (DEBUG / INFO / WARNING / ERROR).
        console: Attach a StreamHandler for stdout output.
        fmt:     'log' → sim.log (human-readable)
                 'csv' → sim.csv (machine-parseable)

    Returns:
        Configured ahrs_sim root logger.
    """
    numeric_level = _LEVEL_MAP.get(level.upper(), logging.INFO)

    root = logging.getLogger(ROOT_LOGGER_NAME)
    root.setLevel(numeric_level)
    root.propagate = False

    # console handler — attach only if not already present
    has_console = any(
        isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler)
        for h in root.handlers
    )
    if console and not has_console:
        sh = logging.StreamHandler()
        sh.setLevel(numeric_level)
        sh.setFormatter(logging.Formatter(_CONSOLE_FMT))
        root.addHandler(sh)

    # file handler — attach only if not already present and log_dir is given
    has_file = any(isinstance(h, logging.FileHandler) for h in root.handlers)
    if log_dir is not None and not has_file:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)

        if fmt == "csv":
            log_path = log_dir / "sim.csv"
            formatter = logging.Formatter(_CSV_FMT)
        else:
            log_path = log_dir / "sim.log"
            formatter = logging.Formatter(_FILE_FMT, datefmt=_DATE_FMT)

        fh = logging.FileHandler(log_path, encoding="utf-8")
        fh.setLevel(numeric_level)
        fh.setFormatter(formatter)
        root.addHandler(fh)
    return root