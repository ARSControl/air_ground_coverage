"""Load one coupled YAML file into aerial and ground parameter views."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .core.base import HEDACParams


@dataclass(frozen=True)
class CoupledConfiguration:
    """Resolved parameter views from one self-contained coupled config."""

    path: Path
    aerial: HEDACParams
    ground: HEDACParams


def load_coupled_configuration(path: str | Path) -> CoupledConfiguration:
    """Load shared settings plus aerial/ground overrides from one YAML file."""
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise TypeError("coupled configuration must be a YAML mapping")

    aerial_overrides = _mapping_section(document, "aerial")
    ground_overrides = _mapping_section(document, "ground")
    shared = {
        key: deepcopy(value)
        for key, value in document.items()
        if key not in {"aerial", "ground"}
    }
    aerial_values = _deep_merge(shared, aerial_overrides)
    ground_values = _deep_merge(shared, ground_overrides)
    return CoupledConfiguration(
        path=config_path,
        aerial=HEDACParams.from_dict(aerial_values),
        ground=HEDACParams.from_dict(ground_values),
    )


def _mapping_section(document: dict[str, Any], name: str) -> dict[str, Any]:
    if name not in document:
        raise ValueError(f"coupled configuration requires a '{name}' section")
    section = document[name]
    if not isinstance(section, dict):
        raise TypeError(f"'{name}' section must be a YAML mapping")
    return section


def _deep_merge(
    shared: dict[str, Any], overrides: dict[str, Any]
) -> dict[str, Any]:
    result = deepcopy(shared)
    for key, value in overrides.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result
