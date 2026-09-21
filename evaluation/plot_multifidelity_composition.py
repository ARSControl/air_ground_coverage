"""Plot reconstruction histories for the composition sweep."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.publication_fonts import configure_publication_fonts  # noqa: E402

configure_publication_fonts()
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from evaluation.multifidelity_composition_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    load_archive,
)


# Paper-oriented defaults. These values target the approximate two-column text
# width of the IEEE conference template used in paper/updated.tex.
DEFAULT_FIGURE_WIDTH = 7.16
DEFAULT_FIGURE_HEIGHT = 2.55
DEFAULT_TITLE_FONT_SIZE = 10.0
DEFAULT_AXIS_LABEL_FONT_SIZE = 9.0
DEFAULT_TICK_LABEL_FONT_SIZE = 8.0
DEFAULT_LEGEND_FONT_SIZE = 9.0
DEFAULT_LINE_WIDTH = 2.2
DEFAULT_BAND_ALPHA = 0.16
DEFAULT_DPI = 300


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_composition_evaluated.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_composition_metrics.png"),
    )
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    style = parser.add_argument_group("paper figure style")
    style.add_argument("--figure-width", type=float, default=DEFAULT_FIGURE_WIDTH)
    style.add_argument("--figure-height", type=float, default=DEFAULT_FIGURE_HEIGHT)
    style.add_argument("--title-font-size", type=float, default=DEFAULT_TITLE_FONT_SIZE)
    style.add_argument(
        "--axis-label-font-size", type=float, default=DEFAULT_AXIS_LABEL_FONT_SIZE
    )
    style.add_argument(
        "--tick-label-font-size", type=float, default=DEFAULT_TICK_LABEL_FONT_SIZE
    )
    style.add_argument(
        "--legend-font-size", type=float, default=DEFAULT_LEGEND_FONT_SIZE
    )
    style.add_argument("--line-width", type=float, default=DEFAULT_LINE_WIDTH)
    style.add_argument("--band-alpha", type=float, default=DEFAULT_BAND_ALPHA)
    style.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    return parser.parse_args()


def plot_composition_evaluation(
    evaluated_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 1000,
    figure_width: float = DEFAULT_FIGURE_WIDTH,
    figure_height: float = DEFAULT_FIGURE_HEIGHT,
    title_font_size: float = DEFAULT_TITLE_FONT_SIZE,
    axis_label_font_size: float = DEFAULT_AXIS_LABEL_FONT_SIZE,
    tick_label_font_size: float = DEFAULT_TICK_LABEL_FONT_SIZE,
    legend_font_size: float = DEFAULT_LEGEND_FONT_SIZE,
    line_width: float = DEFAULT_LINE_WIDTH,
    band_alpha: float = DEFAULT_BAND_ALPHA,
    dpi: int = DEFAULT_DPI,
) -> Path:
    """Render deterministic reconstruction histories for the composition sweep."""
    if isinstance(bootstrap_samples, bool) or bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be a positive integer")
    _validate_style(
        figure_width=figure_width,
        figure_height=figure_height,
        title_font_size=title_font_size,
        axis_label_font_size=axis_label_font_size,
        tick_label_font_size=tick_label_font_size,
        legend_font_size=legend_font_size,
        line_width=line_width,
        band_alpha=band_alpha,
        dpi=dpi,
    )
    _, values = load_archive(evaluated_path, expected_kind=EVALUATED_ARCHIVE_KIND)
    labels = _composition_labels(values)
    colors = plt.get_cmap("viridis")(np.linspace(0.08, 0.92, max(len(labels), 2)))
    rng = np.random.default_rng(20260804)
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(figure_width, figure_height),
        constrained_layout=True,
    )

    _plot_history(
        axes[0],
        values,
        labels,
        colors,
        "nrmse",
        rng,
        bootstrap_samples,
        line_width,
        band_alpha,
    )
    axes[0].set_title("NRMSE", fontsize=title_font_size)
    _plot_history(
        axes[1],
        values,
        labels,
        colors,
        "kl",
        rng,
        bootstrap_samples,
        line_width,
        band_alpha,
    )
    axes[1].set_title("KL Divergence", fontsize=title_font_size)
    _plot_history(
        axes[2],
        values,
        labels,
        colors,
        "nlpd",
        rng,
        bootstrap_samples,
        line_width,
        band_alpha,
    )
    axes[2].set_title("NLPD", fontsize=title_font_size)
    for axis in axes:
        axis.set_xlabel("Mission time [s]", fontsize=axis_label_font_size)
        axis.tick_params(axis="both", labelsize=tick_label_font_size)
        axis.grid(alpha=0.25)

    handles = [
        plt.Line2D([0], [0], color=colors[index], linewidth=line_width, label=label)
        for index, label in enumerate(labels)
    ]
    figure.legend(
        handles=handles,
        loc="outside lower center",
        ncol=len(handles),
        frameon=False,
        fontsize=legend_font_size,
    )
    destination = Path(output_path)
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _validate_style(**values: float | int) -> None:
    for name, value in values.items():
        if name == "band_alpha":
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError("band_alpha must be between zero and one")
        elif isinstance(value, bool) or float(value) <= 0.0:
            raise ValueError(f"{name} must be positive")


def _composition_labels(values: dict[str, np.ndarray]) -> list[str]:
    aerial = np.asarray(values["aerial_counts"], dtype=int)
    ground = np.asarray(values["ground_counts"], dtype=int)
    if aerial.shape != ground.shape or aerial.ndim != 1:
        raise ValueError("aerial_counts and ground_counts must be matching vectors")
    return [
        "aerial-only"
        if ground_count == 0
        else "ground-only"
        if aerial_count == 0
        else "heterogeneous"
        for aerial_count, ground_count in zip(aerial, ground, strict=True)
    ]


def _hyperparameter_policy(metadata: dict) -> str:
    settings = metadata.get("hyperparameter_optimization", {"enabled": False})
    if isinstance(settings, dict) and settings.get("enabled", False):
        return "online optimized"
    return "fixed"


def _plot_history(
    axis,
    values: dict[str, np.ndarray],
    names: list[str],
    colors: np.ndarray,
    metric: str,
    rng: np.random.Generator,
    bootstrap_samples: int,
    line_width: float = DEFAULT_LINE_WIDTH,
    band_alpha: float = DEFAULT_BAND_ALPHA,
) -> None:
    record_compositions = values["posterior_composition"].astype(int)
    record_episodes = values["posterior_episode"].astype(int)
    record_times = values["posterior_time"]
    episode_count = values["seeds"].size
    for composition, name in enumerate(names):
        selected = record_compositions == composition
        times = np.unique(record_times[selected])
        matrix = np.full((episode_count, times.size), np.nan, dtype=float)
        for time_index, timestamp in enumerate(times):
            records = np.flatnonzero(selected & np.isclose(record_times, timestamp))
            matrix[record_episodes[records], time_index] = values[metric][records]
        center, lower, upper = _bootstrap_mean(
            matrix, rng, bootstrap_samples, allow_nan=True
        )
        axis.plot(
            times,
            center,
            color=colors[composition],
            linewidth=line_width,
            label=name,
        )
        axis.fill_between(
            times, lower, upper, color=colors[composition], alpha=band_alpha
        )


def _bootstrap_mean(
    samples: np.ndarray,
    rng: np.random.Generator,
    bootstrap_samples: int,
    *,
    allow_nan: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    matrix = np.asarray(samples, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] < 1:
        raise ValueError("samples must have shape (episodes, values)")
    reducer = np.nanmean if allow_nan else np.mean
    center = reducer(matrix, axis=0)
    indices = rng.integers(
        0, matrix.shape[0], size=(bootstrap_samples, matrix.shape[0])
    )
    resampled = reducer(matrix[indices], axis=1)
    lower, upper = np.nanpercentile(resampled, [2.5, 97.5], axis=0)
    return center, lower, upper


def main() -> None:
    arguments = parse_args()
    path = plot_composition_evaluation(
        arguments.input,
        arguments.output,
        bootstrap_samples=arguments.bootstrap_samples,
        figure_width=arguments.figure_width,
        figure_height=arguments.figure_height,
        title_font_size=arguments.title_font_size,
        axis_label_font_size=arguments.axis_label_font_size,
        tick_label_font_size=arguments.tick_label_font_size,
        legend_font_size=arguments.legend_font_size,
        line_width=arguments.line_width,
        band_alpha=arguments.band_alpha,
        dpi=arguments.dpi,
    )
    print(f"composition_plot={path}")


if __name__ == "__main__":
    main()
