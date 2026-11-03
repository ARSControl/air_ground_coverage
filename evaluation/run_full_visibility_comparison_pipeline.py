#!/usr/bin/env python3
"""Run paired Proposed, Rudolph [17], and Zhang full-visibility experiments."""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import yaml

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.run_baseline_comparison_pipeline import (  # noqa: E402
    DEFAULT_CONFIG,
    DEFAULT_ZHANG_CONFIG,
    run_baseline_comparison_pipeline,
)


DEFAULT_OUTPUT_DIRECTORY = Path("output/full_visibility_comparison")


@dataclass(frozen=True)
class FullVisibilityComparisonArtifacts:
    """Resolved protocol and all three-method comparison outputs."""

    resolved_config: Path
    multifidelity_raw: Path
    rudolph_raw: Path
    zhang_raw: Path
    evaluated: Path
    metrics_plot: Path
    trajectory_plot: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--zhang-config", type=Path, default=DEFAULT_ZHANG_CONFIG)
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=DEFAULT_OUTPUT_DIRECTORY
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument(
        "--full-visibility-range",
        type=float,
        default=None,
        help="shared radius; defaults to the map diagonal and cannot be smaller",
    )
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument(
        "--reuse-raw",
        action="store_true",
        help="reuse all three raw archives and rerun only evaluation and plotting",
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_full_visibility_comparison_pipeline(
    config_path: str | Path,
    zhang_config_path: str | Path,
    output_directory: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    full_visibility_range: float | None = None,
    bootstrap_samples: int = 2000,
    episode: int = 0,
    dpi: int = 180,
    progress: bool = True,
    reuse_raw: bool = False,
) -> FullVisibilityComparisonArtifacts:
    """Resolve a full-visibility scenario and run three exactly paired methods."""
    root = Path(output_directory)
    resolved_config = root / "resolved_full_visibility_config.yaml"
    document = _full_visibility_document(config_path, full_visibility_range)
    _atomic_yaml(resolved_config, document)
    result = run_baseline_comparison_pipeline(
        resolved_config,
        zhang_config_path,
        root,
        episodes=episodes,
        num_steps=num_steps,
        bootstrap_samples=bootstrap_samples,
        episode=episode,
        dpi=dpi,
        progress=progress,
        reuse_raw=reuse_raw,
    )
    return FullVisibilityComparisonArtifacts(
        resolved_config=resolved_config,
        multifidelity_raw=result.multifidelity_raw,
        rudolph_raw=result.egerstedt_raw,
        zhang_raw=result.zhang_raw,
        evaluated=result.evaluated,
        metrics_plot=result.metrics_plot,
        trajectory_plot=result.trajectory_plot,
    )


def _full_visibility_document(
    config_path: str | Path, requested_range: float | None
) -> dict[str, Any]:
    source = Path(config_path)
    with source.open(encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    if not isinstance(loaded, dict):
        raise TypeError("comparison configuration must be a mapping")
    document = deepcopy(loaded)
    size = document.get("map", {}).get("size")
    if (
        not isinstance(size, list)
        or len(size) != 2
        or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in size)
        or any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in size)
    ):
        raise ValueError("map.size must contain two positive finite dimensions")
    diagonal = float(math.hypot(float(size[0]), float(size[1])))
    visibility_range = diagonal if requested_range is None else float(requested_range)
    if not math.isfinite(visibility_range) or visibility_range < diagonal:
        raise ValueError(
            f"full_visibility_range must be at least the map diagonal {diagonal:.6g}"
        )
    obstacle_count = document.get("ground", {}).get("simulation", {}).get(
        "num_obstacles", 0
    )
    if obstacle_count != 0:
        raise ValueError("full-visibility baseline comparison requires zero obstacles")
    for robot_class in ("aerial", "ground"):
        sensor = document.setdefault(robot_class, {}).setdefault("sensor", {})
        sensor["fov_degrees"] = 360.0
        sensor["fov_depth"] = visibility_range
    document["comparison_protocol"] = {
        "name": "proposed_rudolph_zhang_full_visibility",
        "source_config": str(source),
        "full_visibility_range": visibility_range,
        "map_diagonal": diagonal,
        "covers_complete_domain": visibility_range >= diagonal,
        "obstacle_free": True,
        "methods": ["multifidelity", "rudolph_17", "zhang"],
    }
    return document


def _atomic_yaml(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            yaml.safe_dump(document, stream, sort_keys=False)
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main() -> None:
    arguments = parse_args()
    result = run_full_visibility_comparison_pipeline(
        arguments.config,
        arguments.zhang_config,
        arguments.output_dir,
        episodes=arguments.episodes,
        num_steps=arguments.num_steps,
        full_visibility_range=arguments.full_visibility_range,
        bootstrap_samples=arguments.bootstrap_samples,
        episode=arguments.episode,
        dpi=arguments.dpi,
        progress=not arguments.no_progress,
        reuse_raw=arguments.reuse_raw,
    )
    print(f"resolved_config={result.resolved_config}")
    print(f"multifidelity_raw={result.multifidelity_raw}")
    print(f"rudolph_raw={result.rudolph_raw}")
    print(f"zhang_raw={result.zhang_raw}")
    print(f"evaluated={result.evaluated}")
    print(f"metrics_plot={result.metrics_plot}")
    print(f"trajectory_plot={result.trajectory_plot}")


if __name__ == "__main__":
    main()
