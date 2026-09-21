"""Run, evaluate, and plot the paired three-method baseline comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Callable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.evaluate_three_method_comparison import (  # noqa: E402
    evaluate_three_method_comparison,
)
from evaluation.plot_three_method_comparison import (  # noqa: E402
    plot_three_method_comparison,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.run_egerstedt_comparison import run_egerstedt_comparison  # noqa: E402
from evaluation.run_multifidelity_comparison import (  # noqa: E402
    run_multifidelity_comparison,
)
from evaluation.run_zhang_comparison import run_zhang_comparison  # noqa: E402


DEFAULT_CONFIG = Path("configs/baseline_comparison_a3_g7.yaml")
DEFAULT_ZHANG_CONFIG = Path("configs/zhang_comparison.yaml")
DEFAULT_OUTPUT_DIRECTORY = Path("output/three_method_comparison")


@dataclass(frozen=True)
class BaselineComparisonArtifacts:
    """Complete independent artifact set produced by the pipeline."""

    multifidelity_raw: Path
    egerstedt_raw: Path
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


def run_baseline_comparison_pipeline(
    config_path: str | Path,
    zhang_config_path: str | Path,
    output_directory: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    bootstrap_samples: int = 2000,
    episode: int = 0,
    dpi: int = 180,
    progress: bool = True,
    reuse_raw: bool = False,
) -> BaselineComparisonArtifacts:
    """Execute the independent runners followed by common offline analysis."""
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
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    if not isinstance(progress, bool) or not isinstance(reuse_raw, bool):
        raise TypeError("progress and reuse_raw must be booleans")

    root = Path(output_directory)
    multifidelity_raw = root / "multifidelity" / "raw.npz"
    egerstedt_raw = root / "egerstedt" / "raw.npz"
    zhang_raw = root / "zhang" / "raw.npz"
    evaluated = root / "evaluated.npz"
    if reuse_raw:
        for path in (multifidelity_raw, egerstedt_raw, zhang_raw):
            if not path.is_file():
                raise FileNotFoundError(f"cannot reuse missing raw archive: {path}")
    else:
        _run_with_progress(
            "Proposed",
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
            "Egerstedt",
            progress,
            lambda callback: run_egerstedt_comparison(
                multifidelity_raw,
                egerstedt_raw,
                progress_callback=callback,
            ),
        )
        _run_with_progress(
            "Zhang",
            progress,
            lambda callback: run_zhang_comparison(
                multifidelity_raw,
                zhang_raw,
                config_path=zhang_config_path,
                progress_callback=callback,
            ),
        )
    _run_with_progress(
        "Evaluate",
        progress,
        lambda callback: evaluate_three_method_comparison(
            multifidelity_raw,
            egerstedt_raw,
            zhang_raw,
            evaluated,
            bootstrap_samples=bootstrap_samples,
            progress_callback=callback,
        ),
    )
    metrics_plot, trajectory_plot = plot_three_method_comparison(
        evaluated,
        multifidelity_raw,
        egerstedt_raw,
        zhang_raw,
        root / "plots",
        episode=episode,
        dpi=dpi,
    )
    return BaselineComparisonArtifacts(
        multifidelity_raw=multifidelity_raw,
        egerstedt_raw=egerstedt_raw,
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
    result = run_baseline_comparison_pipeline(
        arguments.config,
        arguments.zhang_config,
        arguments.output_dir,
        episodes=arguments.episodes,
        num_steps=arguments.num_steps,
        bootstrap_samples=arguments.bootstrap_samples,
        episode=arguments.episode,
        dpi=arguments.dpi,
        progress=not arguments.no_progress,
        reuse_raw=arguments.reuse_raw,
    )
    print(f"multifidelity_raw={result.multifidelity_raw}")
    print(f"egerstedt_raw={result.egerstedt_raw}")
    print(f"zhang_raw={result.zhang_raw}")
    print(f"three_method_evaluated={result.evaluated}")
    print(f"three_method_metrics_plot={result.metrics_plot}")
    print(f"three_method_trajectory_plot={result.trajectory_plot}")


if __name__ == "__main__":
    main()
