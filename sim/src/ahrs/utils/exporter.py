"""
File Name: ./src/ahrs/utils/exporter.py
Author: Beomjun Chung
Updated: 2026-05-27

Description:
    Generic file I/O utilities and output directory management.

    ExportContext  — typed container for all output directory paths
    build_export   — create directory tree from config, return ExportContext
    save_csv       — write tabular data (ndarray or row-iterable) to CSV
    save_json      — serialize dict (numpy-safe) to JSON
    save_figure    — save a matplotlib Figure to a file
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

_DEFAULT_SUBDIRS: dict[str, str] = {
    "data_dir":   "data",
    "fig_dir":    "fig",
    "log_dir":    "log",
    "report_dir": "report",
}


@dataclass
class ExportContext:
    export_dir: Path
    data_dir:   Path
    fig_dir:    Path
    log_dir:    Path
    report_dir: Path

    @classmethod
    def from_export_dir(
        cls,
        export_dir: Path,
        subdirs: Mapping[str, str] | None = None,
    ) -> "ExportContext":
        subdir_map = dict(_DEFAULT_SUBDIRS)
        if subdirs is not None:
            subdir_map.update(subdirs)

        paths: dict[str, Path] = {"export_dir": export_dir}
        for attr_name, folder_name in subdir_map.items():
            path = export_dir / folder_name
            path.mkdir(parents=True, exist_ok=True)
            paths[attr_name] = path

        return cls(**paths)


def build_export(cfg: dict) -> ExportContext:
    """
    Create the run output directory tree from config and return ExportContext.

    Config key: cfg["export"]
        root_dir       (str)  — root export folder, default "export"
        timestamp_dirs (bool) — append YYYYMMDD_HHMMSS subfolder, default true
    """
    out_cfg  = cfg.get("export", {})
    root_dir = Path(out_cfg.get("root_dir", out_cfg.get("base_dir", "export")))
    use_ts   = out_cfg.get("timestamp_dirs", True)

    if use_ts:
        ts         = datetime.now().strftime("%Y%m%d_%H%M%S")
        export_dir = root_dir / ts
    else:
        export_dir = root_dir / "latest"

    subdirs = out_cfg.get("subdirs")
    return ExportContext.from_export_dir(export_dir, subdirs)


def save_csv(path: Path, headers: Sequence[str], rows) -> None:
    """Write tabular data to CSV.

    Args:
        path:    output file path (parent directories are created if needed)
        headers: column header names
        rows:    2-D ndarray (shape N×C) or iterable of row sequences
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(headers)
        if isinstance(rows, np.ndarray):
            w.writerows(rows.tolist())
        else:
            w.writerows(rows)


def save_json(path: Path, data: dict) -> None:
    """Serialize dict (converts numpy arrays/scalars) to JSON file."""
    def _convert(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        if isinstance(obj, dict):
            return {k: _convert(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_convert(i) for i in obj]
        return obj

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(_convert(data), f, indent=2, ensure_ascii=False)


def save_figure(fig, path: Path, dpi: int = 150) -> None:
    """Save a matplotlib Figure to a file.

    Args:
        fig:  matplotlib Figure object
        path: output file path; format inferred from extension
        dpi:  resolution (default 150)
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
