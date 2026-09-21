"""Run the paired limited/full-range four-condition publication comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Callable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.evaluate_four_method_comparison import (  # noqa: E402
    evaluate_four_method_comparison,
)
from evaluation.plot_four_method_comparison import (  # noqa: E402
    plot_four_method_comparison,
)
from evaluation.plot_four_method_trajectories import (  # noqa: E402
    plot_four_method_trajectories,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.run_egerstedt_comparison import run_egerstedt_comparison  # noqa: E402
from evaluation.run_multifidelity_comparison import (  # noqa: E402
    run_multifidelity_comparison,
)
from evaluation.run_zhang_comparison import run_zhang_comparison  # noqa: E402


DEFAULT_CONFIG = Path("configs/baseline_comparison_a3_g7.yaml")
DEFAULT_ZHANG_CONFIG = Path("configs/zhang_comparison.yaml")
DEFAULT_OUTPUT_DIRECTORY = Path("output/four_method_comparison")
DEFAULT_EGERSTEDT_FULL_RANGE = 75.0


@dataclass(frozen=True)
class FourMethodComparisonArtifacts:
    """Complete four-condition raw, evaluated, and plot artifact set."""

    multifidelity_raw: Path
    egerstedt_lr_raw: Path
    egerstedt_full_raw: Path
    zhang_raw: Path
    evaluated: Path
    metrics_plot: Path
    trajectory_plot: Path | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--zhang-config", type=Path, default=DEFAULT_ZHANG_CONFIG)
    parser.add_argument(
        "--egerstedt-full-range",
        type=float,
        default=DEFAULT_EGERSTEDT_FULL_RANGE,
        help="effective Egerstedt radius that must cover the complete domain",
    )
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=DEFAULT_OUTPUT_DIRECTORY
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument(
        "--episode",
        type=int,
        default=None,
        help="also plot this zero-based representative episode",
    )
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument(
        "--reuse-raw",
        action="store_true",
        help="reuse all four raw archives and rerun only evaluation and plotting",
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_four_method_comparison_pipeline(
    config_path: str | Path,
    zhang_config_path: str | Path,
    output_directory: str | Path,
    *,
    egerstedt_full_range: float = DEFAULT_EGERSTEDT_FULL_RANGE,
    episodes: int | None = None,
    num_steps: int | None = None,
    bootstrap_samples: int = 2000,
    episode: int | None = None,
    dpi: int = 300,
    progress: bool = True,
    reuse_raw: bool = False,
) -> FourMethodComparisonArtifacts:
    """Run four paired conditions and create only the requested metric figure."""
    if episodes is not None and (
        isinstance(episodes, bool) or not isinstance(episodes, int) or episodes < 1
    ):
        raise ValueError("episodes must be a positive integer or None")
    if num_steps is not None and (
        isinstance(num_steps, bool) or not isinstance(num_steps, int) or num_steps < 1
    ):
        raise ValueError("num_steps must be a positive integer or None")
    if (
        isinstance(bootstrap_samples, bool)
        or not isinstance(bootstrap_samples, int)
        or bootstrap_samples < 1
    ):
        raise ValueError("bootstrap_samples must be a positive integer")
    if episode is not None and (
        isinstance(episode, bool) or not isinstance(episode, int) or episode < 0
    ):
        raise ValueError("episode must be a nonnegative integer or None")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    if not isinstance(progress, bool) or not isinstance(reuse_raw, bool):
        raise TypeError("progress and reuse_raw must be booleans")

    root = Path(output_directory)
    multifidelity_raw = root / "multifidelity" / "raw.npz"
    egerstedt_lr_raw = root / "egerstedt_lr" / "raw.npz"
    egerstedt_full_raw = root / "egerstedt_full" / "raw.npz"
    zhang_raw = root / "zhang" / "raw.npz"
    evaluated = root / "evaluated.npz"
    metrics_plot = root / "plots" / "coverage_locational_cost.png"
    trajectory_plot = (
        None
        if episode is None
        else root / "plots" / f"trajectories_episode_{episode:03d}.png"
    )
    raw_paths = (
        multifidelity_raw,
        egerstedt_lr_raw,
        egerstedt_full_raw,
        zhang_raw,
    )
    if reuse_raw:
        for path in raw_paths:
            if not path.is_file():
                raise FileNotFoundError(f"cannot reuse missing raw archive: {path}")
    else:
        _run_with_progress(
            "Proposed-LR",
            progress,
            lambda callback: run_multifidelity_comparison(
                config_path,
                multifidelity_raw,
                episodes=episodes,
                num_steps=num_steps,
                progress_callback=callback,
            ),
        )
        _run_with_progress(
            "Egerstedt-LR",
            progress,
            lambda callback: run_egerstedt_comparison(
                multifidelity_raw,
                egerstedt_lr_raw,
                progress_callback=callback,
            ),
        )
        _run_with_progress(
            "Egerstedt full-range",
            progress,
            lambda callback: run_egerstedt_comparison(
                multifidelity_raw,
                egerstedt_full_raw,
                aerial_sensing_range=egerstedt_full_range,
                progress_callback=callback,
            ),
        )
        _run_with_progress(
            "Zhang full-cell",
            progress,
            lambda callback: run_zhang_comparison(
                multifidelity_raw,
                zhang_raw,
                config_path=zhang_config_path,
                progress_callback=callback,
            ),
        )
    _run_with_progress(
        "Evaluate four conditions",
        progress,
        lambda callback: evaluate_four_method_comparison(
            multifidelity_raw,
            egerstedt_lr_raw,
            egerstedt_full_raw,
            zhang_raw,
            evaluated,
            bootstrap_samples=bootstrap_samples,
            progress_callback=callback,
        ),
    )
    plot_four_method_comparison(evaluated, metrics_plot, dpi=dpi)
    if trajectory_plot is not None and episode is not None:
        plot_four_method_trajectories(
            root,
            trajectory_plot,
            episode=episode,
            dpi=dpi,
        )
    return FourMethodComparisonArtifacts(
        multifidelity_raw=multifidelity_raw,
        egerstedt_lr_raw=egerstedt_lr_raw,
        egerstedt_full_raw=egerstedt_full_raw,
        zhang_raw=zhang_raw,
        evaluated=evaluated,
        metrics_plot=metrics_plot,
        trajectory_plot=trajectory_plot,
    )


def _run_with_progress(
    label: str,
    enabled: bool,
    operation: Callable[[Callable[[int, int, str], None] | None], object],
) -> None:
    display = TerminalProgress(label) if enabled else None
    try:
        operation(None if display is None else display.update)
    finally:
        if display is not None:
            display.close()


def main() -> None:
    arguments = parse_args()
    result = run_four_method_comparison_pipeline(
        arguments.config,
        arguments.zhang_config,
        arguments.output_dir,
        egerstedt_full_range=arguments.egerstedt_full_range,
        episodes=arguments.episodes,
        num_steps=arguments.num_steps,
        bootstrap_samples=arguments.bootstrap_samples,
        episode=arguments.episode,
        dpi=arguments.dpi,
        progress=not arguments.no_progress,
        reuse_raw=arguments.reuse_raw,
    )
    print(f"multifidelity_lr_raw={result.multifidelity_raw}")
    print(f"egerstedt_lr_raw={result.egerstedt_lr_raw}")
    print(f"egerstedt_full_raw={result.egerstedt_full_raw}")
    print(f"zhang_full_raw={result.zhang_raw}")
    print(f"four_method_evaluated={result.evaluated}")
    print(f"four_method_metrics_plot={result.metrics_plot}")
    if result.trajectory_plot is not None:
        print(f"four_method_trajectory_plot={result.trajectory_plot}")


if __name__ == "__main__":
    main()
