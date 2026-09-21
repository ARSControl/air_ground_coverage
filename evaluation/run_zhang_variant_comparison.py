"""Run and plot standard Zhang versus Zhang-LR on one paired scenario."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Callable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.evaluate_zhang_variant_comparison import (  # noqa: E402
    evaluate_zhang_variant_comparison,
)
from evaluation.plot_zhang_variant_comparison import (  # noqa: E402
    plot_zhang_variant_comparison,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.run_multifidelity_comparison import (  # noqa: E402
    run_multifidelity_comparison,
)
from evaluation.run_zhang_comparison import run_zhang_comparison  # noqa: E402


DEFAULT_CONFIG = Path("configs/baseline_comparison_a3_g7.yaml")
DEFAULT_STANDARD_CONFIG = Path("configs/zhang_comparison.yaml")
DEFAULT_LR_CONFIG = Path("configs/zhang_lr_comparison.yaml")
DEFAULT_OUTPUT_DIRECTORY = Path("output/zhang_variant_comparison")


@dataclass(frozen=True)
class ZhangVariantComparisonArtifacts:
    """Complete direct-comparison artifact set."""

    scenario: Path
    standard_raw: Path
    lr_raw: Path
    evaluated: Path
    plot: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--standard-config", type=Path, default=DEFAULT_STANDARD_CONFIG)
    parser.add_argument("--lr-config", type=Path, default=DEFAULT_LR_CONFIG)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
        help="reuse an existing proposed-method raw scenario instead of generating one",
    )
    parser.add_argument(
        "--output-dir", "-o", type=Path, default=DEFAULT_OUTPUT_DIRECTORY
    )
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_zhang_variant_comparison(
    config_path: str | Path,
    output_directory: str | Path,
    *,
    standard_config_path: str | Path = DEFAULT_STANDARD_CONFIG,
    lr_config_path: str | Path = DEFAULT_LR_CONFIG,
    scenario_path: str | Path | None = None,
    episodes: int = 1,
    num_steps: int | None = None,
    episode: int = 0,
    dpi: int = 180,
    progress: bool = True,
) -> ZhangVariantComparisonArtifacts:
    """Generate one scenario, run both Zhang variants, evaluate, and plot."""
    if isinstance(episodes, bool) or not isinstance(episodes, int) or episodes < 1:
        raise ValueError("episodes must be a positive integer")
    if num_steps is not None and (
        isinstance(num_steps, bool) or not isinstance(num_steps, int) or num_steps < 1
    ):
        raise ValueError("num_steps must be a positive integer or None")
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    if not isinstance(progress, bool):
        raise TypeError("progress must be a boolean")

    root = Path(output_directory)
    scenario = (
        root / "scenario" / "raw.npz" if scenario_path is None else Path(scenario_path)
    )
    standard_raw = root / "standard" / "raw.npz"
    lr_raw = root / "lr" / "raw.npz"
    evaluated = root / "evaluated.npz"
    plot = root / "comparison.png"
    if scenario_path is None:
        _run_with_progress(
            "Scenario",
            progress,
            lambda callback: run_multifidelity_comparison(
                config_path,
                scenario,
                episodes=episodes,
                num_steps=num_steps,
                progress_callback=callback,
            ),
        )
    elif not scenario.is_file():
        raise FileNotFoundError(f"scenario archive does not exist: {scenario}")

    _run_with_progress(
        "Standard Zhang",
        progress,
        lambda callback: run_zhang_comparison(
            scenario,
            standard_raw,
            config_path=standard_config_path,
            progress_callback=callback,
        ),
    )
    _run_with_progress(
        "Zhang-LR",
        progress,
        lambda callback: run_zhang_comparison(
            scenario,
            lr_raw,
            config_path=lr_config_path,
            progress_callback=callback,
        ),
    )
    _run_with_progress(
        "Evaluate variants",
        progress,
        lambda callback: evaluate_zhang_variant_comparison(
            standard_raw,
            lr_raw,
            evaluated,
            progress_callback=callback,
        ),
    )
    plot_zhang_variant_comparison(
        evaluated,
        standard_raw,
        lr_raw,
        plot,
        episode=episode,
        dpi=dpi,
    )
    return ZhangVariantComparisonArtifacts(
        scenario=scenario,
        standard_raw=standard_raw,
        lr_raw=lr_raw,
        evaluated=evaluated,
        plot=plot,
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
    result = run_zhang_variant_comparison(
        arguments.config,
        arguments.output_dir,
        standard_config_path=arguments.standard_config,
        lr_config_path=arguments.lr_config,
        scenario_path=arguments.scenario,
        episodes=arguments.episodes,
        num_steps=arguments.num_steps,
        episode=arguments.episode,
        dpi=arguments.dpi,
        progress=not arguments.no_progress,
    )
    print(f"scenario={result.scenario}")
    print(f"standard_zhang_raw={result.standard_raw}")
    print(f"zhang_lr_raw={result.lr_raw}")
    print(f"zhang_variant_evaluated={result.evaluated}")
    print(f"zhang_variant_plot={result.plot}")


if __name__ == "__main__":
    main()
