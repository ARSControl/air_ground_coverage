"""Run, evaluate, and plot the complete multi-fidelity scenario comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.evaluate_multifidelity_composition import (  # noqa: E402
    evaluate_composition_archive,
)
from evaluation.plot_multifidelity_composition import (  # noqa: E402
    plot_composition_evaluation,
)
from evaluation.plot_multifidelity_composition_trajectories import (  # noqa: E402
    plot_composition_trajectories,
)
from evaluation.plot_multifidelity_scenario_comparison import (  # noqa: E402
    plot_scenario_comparison,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.run_multifidelity_composition import (  # noqa: E402
    run_composition_sweep,
)
from src.coupled_config import load_coupled_configuration  # noqa: E402


DEFAULT_CONFIG = Path("configs/multifidelity_composition.yaml")
DEFAULT_OUTPUT_DIRECTORY = Path("output/multifidelity_scenario_pipeline")


@dataclass(frozen=True)
class ScenarioArtifacts:
    """Files produced for one declared scenario."""

    scenario: str
    raw_archive: Path
    evaluated_archive: Path
    metrics_plot: Path
    trajectory_plot: Path


@dataclass(frozen=True)
class ScenarioPipelineResult:
    """Complete output set from one two-scenario pipeline execution."""

    scenarios: tuple[ScenarioArtifacts, ScenarioArtifacts]
    comparison_metrics_plot: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=DEFAULT_OUTPUT_DIRECTORY
    )
    parser.add_argument(
        "--composition",
        required=True,
        help="exact composition label to use for trajectory plots, e.g. A2/G8",
    )
    parser.add_argument(
        "--episode",
        type=int,
        default=0,
        help="zero-based saved episode index for trajectory plots (default: 0)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=None,
        help="override simulation.num_episodes for every declared scenario",
    )
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument(
        "--reuse-raw",
        action="store_true",
        help=(
            "reuse existing raw/<scenario>.npz archives and rerun only "
            "evaluation and plotting"
        ),
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress bars"
    )
    return parser.parse_args()


def run_scenario_pipeline(
    config_path: str | Path,
    output_directory: str | Path,
    *,
    composition: str,
    episode: int = 0,
    episodes: int | None = None,
    bootstrap_samples: int = 1000,
    dpi: int = 180,
    progress: bool = True,
    reuse_raw: bool = False,
) -> ScenarioPipelineResult:
    """Run or reuse both scenarios and produce all archives and figures."""
    if not isinstance(composition, str) or not composition.strip():
        raise TypeError("composition must be a nonempty string")
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    if episodes is not None and (
        isinstance(episodes, bool) or not isinstance(episodes, int) or episodes < 1
    ):
        raise ValueError("episodes must be a positive integer or None")
    if (
        isinstance(bootstrap_samples, bool)
        or not isinstance(bootstrap_samples, int)
        or bootstrap_samples < 1
    ):
        raise ValueError("bootstrap_samples must be a positive integer")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    if not isinstance(progress, bool):
        raise TypeError("progress must be a boolean")
    if not isinstance(reuse_raw, bool):
        raise TypeError("reuse_raw must be a boolean")

    configuration = load_coupled_configuration(config_path)
    if not configuration.aerial.get(
        "multifidelity.hyperparameter_optimization.enabled", False
    ):
        raise ValueError(
            "scenario pipeline requires hyperparameter optimization to be enabled"
        )
    declared = configuration.aerial.get("composition_sweep.scenarios", None)
    if not isinstance(declared, dict) or len(declared) != 2:
        raise ValueError(
            "scenario pipeline requires exactly two named composition_sweep scenarios"
        )
    scenario_names = tuple(str(name) for name in declared)
    episode_count = (
        int(configuration.aerial.num_episodes) if episodes is None else episodes
    )
    if episode >= episode_count:
        raise IndexError(
            f"episode {episode} is outside the configured {episode_count} episodes"
        )

    root = Path(output_directory)
    raw_directory = root / "raw"
    evaluated_directory = root / "evaluated"
    plot_directory = root / "plots"
    safe_composition = _safe_name(composition.strip())
    artifacts: list[ScenarioArtifacts] = []

    for scenario in scenario_names:
        safe_scenario = _safe_name(scenario)
        raw_path = raw_directory / f"{safe_scenario}.npz"
        evaluated_path = evaluated_directory / f"{safe_scenario}.npz"
        metrics_path = plot_directory / f"{safe_scenario}_metrics.png"
        trajectory_path = plot_directory / (
            f"{safe_scenario}_{safe_composition}_episode_{episode:03d}.png"
        )

        if reuse_raw:
            if not raw_path.is_file():
                raise FileNotFoundError(
                    f"cannot reuse missing raw scenario archive: {raw_path}"
                )
        else:
            run_progress = TerminalProgress(f"Run {scenario}") if progress else None
            try:
                run_composition_sweep(
                    config_path,
                    raw_path,
                    episodes=episode_count,
                    scenario=scenario,
                    progress_callback=(
                        None if run_progress is None else run_progress.update
                    ),
                )
            finally:
                if run_progress is not None:
                    run_progress.close()

        evaluation_progress = (
            TerminalProgress(f"Evaluate {scenario}") if progress else None
        )
        try:
            evaluate_composition_archive(
                raw_path,
                evaluated_path,
                progress_callback=(
                    None if evaluation_progress is None else evaluation_progress.update
                ),
            )
        finally:
            if evaluation_progress is not None:
                evaluation_progress.close()

        plot_composition_evaluation(
            evaluated_path,
            metrics_path,
            bootstrap_samples=bootstrap_samples,
        )
        plot_composition_trajectories(
            raw_path,
            composition=composition,
            episode=episode,
            output_path=trajectory_path,
            dpi=dpi,
        )
        artifacts.append(
            ScenarioArtifacts(
                scenario=scenario,
                raw_archive=raw_path,
                evaluated_archive=evaluated_path,
                metrics_plot=metrics_path,
                trajectory_plot=trajectory_path,
            )
        )

    comparison_path = plot_directory / "scenario_comparison_metrics.png"
    plot_scenario_comparison(
        artifacts[0].evaluated_archive,
        artifacts[1].evaluated_archive,
        comparison_path,
        bootstrap_samples=bootstrap_samples,
    )
    return ScenarioPipelineResult(
        scenarios=(artifacts[0], artifacts[1]),
        comparison_metrics_plot=comparison_path,
    )


def _safe_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_-]+", "_", value).strip("_")
    if not result:
        raise ValueError("output label must contain a filename-safe character")
    return result


def main() -> None:
    arguments = parse_args()
    result = run_scenario_pipeline(
        arguments.config,
        arguments.output_dir,
        composition=arguments.composition,
        episode=arguments.episode,
        episodes=arguments.episodes,
        bootstrap_samples=arguments.bootstrap_samples,
        dpi=arguments.dpi,
        progress=not arguments.no_progress,
        reuse_raw=arguments.reuse_raw,
    )
    for artifact in result.scenarios:
        print(f"scenario={artifact.scenario}")
        print(f"raw_archive={artifact.raw_archive}")
        print(f"evaluated_archive={artifact.evaluated_archive}")
        print(f"metrics_plot={artifact.metrics_plot}")
        print(f"trajectory_plot={artifact.trajectory_plot}")
    print(f"scenario_comparison_plot={result.comparison_metrics_plot}")


if __name__ == "__main__":
    main()
