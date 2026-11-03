"""Run, evaluate, and plot proposed-method-only parameter sensitivity sweeps."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_parameter_sweep import (  # noqa: E402
    SUPPORTED_FACTORS,
    evaluate_parameter_condition,
    load_sweep_definition,
    plot_parameter_factor,
    resolve_conditions,
    write_manifest,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.run_multifidelity_comparison import (  # noqa: E402
    run_multifidelity_comparison,
)


@dataclass(frozen=True)
class ParameterSweepArtifacts:
    manifest: Path
    raw_archives: tuple[Path, ...]
    evaluated_archives: tuple[Path, ...]
    plots: tuple[Path, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        "-c",
        type=Path,
        default=Path("configs/multifidelity_parameter_sweep.yaml"),
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=Path("output/multifidelity_parameter_sweep"),
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--factors", nargs="+", choices=SUPPORTED_FACTORS, default=None)
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    parser.add_argument("--reuse-raw", action="store_true")
    parser.add_argument("--no-progress", action="store_true")
    return parser.parse_args()


def run_parameter_sweep_pipeline(
    config_path: str | Path,
    output_directory: str | Path,
    *,
    factors: tuple[str, ...] | list[str] | None = None,
    episodes: int | None = None,
    bootstrap_samples: int = 1000,
    reuse_raw: bool = False,
    progress: bool = True,
) -> ParameterSweepArtifacts:
    """Run selected OFAT conditions using only the proposed MFGP method."""
    definition = load_sweep_definition(config_path)
    selected = SUPPORTED_FACTORS if factors is None else tuple(factors)
    unknown = set(selected).difference(SUPPORTED_FACTORS)
    if unknown:
        raise ValueError(f"unknown factors: {sorted(unknown)}")
    if not selected or len(set(selected)) != len(selected):
        raise ValueError("factors must be nonempty and unique")
    if episodes is not None and (
        isinstance(episodes, bool) or not isinstance(episodes, int) or episodes < 1
    ):
        raise ValueError("episodes must be a positive integer or None")
    if bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be positive")
    root = Path(output_directory)
    conditions = tuple(
        item
        for item in resolve_conditions(definition, root / "configs")
        if item.factor in selected
    )
    raw_paths: list[Path] = []
    evaluated_paths: list[Path] = []
    records: list[dict[str, object]] = []
    for condition in conditions:
        raw = root / "raw" / condition.factor / f"{condition.slug}.npz"
        evaluated = root / "evaluated" / condition.factor / f"{condition.slug}.npz"
        if reuse_raw:
            if not raw.is_file():
                raise FileNotFoundError(f"cannot reuse missing raw archive: {raw}")
        else:
            meter = TerminalProgress(f"Run {condition.slug}") if progress else None
            try:
                run_multifidelity_comparison(
                    condition.config_path,
                    raw,
                    episodes=episodes,
                    progress_callback=None if meter is None else meter.update,
                )
            finally:
                if meter is not None:
                    meter.close()
        evaluate_parameter_condition(raw, evaluated)
        raw_paths.append(raw)
        evaluated_paths.append(evaluated)
        records.append(
            {
                "factor": condition.factor,
                "label": condition.label,
                "reference": next(
                    level.get("reference") is True
                    for level in definition.factors[condition.factor]
                    if level["label"] == condition.label
                ),
                "slug": condition.slug,
                "resolved_value": condition.resolved_value,
                "config": str(condition.config_path.relative_to(root)),
                "raw_archive": str(raw.relative_to(root)),
                "evaluated_archive": str(evaluated.relative_to(root)),
            }
        )
    manifest_document = {
        "method": "multifidelity",
        "baselines_included": False,
        "design": "one_factor_at_a_time",
        "source_config": str(Path(config_path)),
        "base_config": str(definition.base_config_path),
        "selected_factors": list(selected),
        "conditions": records,
    }
    manifest = write_manifest(root / "manifest.json", manifest_document)
    plots: list[Path] = []
    for factor in selected:
        factor_records = [item for item in records if item["factor"] == factor]
        plot = root / "plots" / f"{factor}.png"
        plot_parameter_factor(
            factor,
            factor_records,
            root,
            plot,
            bootstrap_samples=bootstrap_samples,
        )
        plots.append(plot)
    return ParameterSweepArtifacts(
        manifest,
        tuple(raw_paths),
        tuple(evaluated_paths),
        tuple(plots),
    )


def main() -> None:
    arguments = parse_args()
    result = run_parameter_sweep_pipeline(
        arguments.config,
        arguments.output_dir,
        factors=arguments.factors,
        episodes=arguments.episodes,
        bootstrap_samples=arguments.bootstrap_samples,
        reuse_raw=arguments.reuse_raw,
        progress=not arguments.no_progress,
    )
    print(f"manifest={result.manifest}")
    for plot in result.plots:
        print(f"plot={plot}")


if __name__ == "__main__":
    main()
