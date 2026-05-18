"""
File Name: ./src/ahrs/utils/configger.py
Author: Beomjun Chung
Updated: 2026-05-18

Description:
  YAML 설정 파일 로딩 및 검증

    purpose:
        config.yaml을 읽어 dict로 반환. 필수 키 누락 시 명확한 오류 메시지 제공.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

_REQUIRED_KEYS = ["simulation", "env", "trajectory", "imu", "filters", "output"]

def load_config(path: str | Path) -> dict[str, Any]:
    """
    config.yaml 로딩.

    Args:
        path: config.yaml 경로

    Returns:
        config dict

    Raises:
        FileNotFoundError: 파일이 없으면
        KeyError: 필수 키가 없으면
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if cfg is None:
        raise ValueError(f"Config file is empty: {path}")

    for key in _REQUIRED_KEYS:
        if key not in cfg:
            raise KeyError(f"Required config key missing: '{key}'")

    return cfg

def get_nested(cfg: dict, *keys: str, default: Any = None) -> Any:
    """중첩 dict에서 안전하게 값 추출."""
    cur = cfg
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur