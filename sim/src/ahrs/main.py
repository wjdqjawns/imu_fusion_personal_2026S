"""
File Name: ./src/ahrs/main.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  Main entry point for AHRS simulation and analysis.

    Example:
        py -m ahrs.main --config config/config.yaml
        py -m ahrs.main --config config/config.yaml --dry-run --log-level DEBUG
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from ahrs.pipeline.runner import run

def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ahrs",
        description="IMU Fusion Simulation CLI",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/config.yaml"),
        help="Path to config.yaml",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate config without running simulation",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    )
    return parser

def cli_main(argv: Sequence[str] | None = None) -> None:
    args = _build_parser().parse_args(argv)

    run(
        config_path=args.config,
        dry_run=args.dry_run,
        log_level=args.log_level,
    )

if __name__ == "__main__":
    cli_main()