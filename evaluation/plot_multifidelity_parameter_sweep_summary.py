#!/usr/bin/env python3
"""Create combined metric summaries from a completed parameter sweep."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_parameter_sweep import (  # noqa: E402
    plot_parameter_metric_effects,
    plot_parameter_metric_overview,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--metric", default="final_kl")
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    return parser.parse_args()


def main() -> None:
    arguments = parse_args()
    with arguments.manifest.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    root = arguments.manifest.parent
    output = arguments.output_dir or root / "plots"
    safe_metric = arguments.metric.replace("/", "_").replace("\\", "_")
    overview = plot_parameter_metric_overview(
        manifest["conditions"],
        root,
        output / f"all_parameters__{safe_metric}.png",
        metric=arguments.metric,
        bootstrap_samples=arguments.bootstrap_samples,
    )
    effects = plot_parameter_metric_effects(
        manifest["conditions"],
        root,
        output / f"all_parameters__{safe_metric}__paired_effects.png",
        metric=arguments.metric,
        bootstrap_samples=arguments.bootstrap_samples,
    )
    print(f"overview={overview}")
    print(f"effects={effects}")


if __name__ == "__main__":
    main()
