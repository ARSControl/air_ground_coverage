"""Plot only coverage and locational cost for the four paired conditions."""

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

from evaluation.comparison_io import (  # noqa: E402
    FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    load_archive,
)
from evaluation.plot_multifidelity_composition import (  # noqa: E402
    DEFAULT_AXIS_LABEL_FONT_SIZE,
    DEFAULT_BAND_ALPHA,
    DEFAULT_LEGEND_FONT_SIZE,
    DEFAULT_LINE_WIDTH,
    DEFAULT_TICK_LABEL_FONT_SIZE,
    DEFAULT_TITLE_FONT_SIZE,
)


METHODS = ("multifidelity_lr", "egerstedt_lr", "egerstedt_full", "zhang_full")
DEFAULT_FIGURE_WIDTH = 3.5
DEFAULT_FIGURE_HEIGHT = 3.0
DEFAULT_DPI = 300
LABELS = {
    "multifidelity_lr": "Proposed",
    "egerstedt_lr": "Rudolph [17]-LR",
    "egerstedt_full": "Rudolph [17]",
    "zhang_full": "Zhang [18]",
}
COLORS = {
    "multifidelity_lr": "#0072B2",
    "egerstedt_lr": "#E69F00",
    "egerstedt_full": "#D55E00",
    "zhang_full": "#009E73",
}
LINESTYLES = {
    "multifidelity_lr": "-",
    "egerstedt_lr": "--",
    "egerstedt_full": "-",
    "zhang_full": "-",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/four_method_comparison")
    parser.add_argument("--evaluated", type=Path, default=root / "evaluated.npz")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=root / "plots/coverage_locational_cost.png",
    )
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


def plot_four_method_comparison(
    evaluated_path: str | Path,
    output_path: str | Path,
    *,
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
    """Render the two requested time-history panels and nothing else."""
    metadata, values = load_archive(
        evaluated_path,
        expected_kind=FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    )
    if metadata.get("methods") != list(METHODS):
        raise ValueError("evaluated archive does not contain the four expected methods")
    if metadata.get("final_plot_metrics") != [
        "footprint_normalized_coverage",
        "locational_cost",
    ]:
        raise ValueError(
            "evaluated archive does not declare the requested plot metrics"
        )
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, (coverage_axis, cost_axis) = plt.subplots(
        1,
        2,
        figsize=(figure_width, figure_height),
        constrained_layout=True,
    )
    times = values["state_times"]
    labels = _labels(values["effective_aerial_sensing_ranges"])
    _plot_histories(
        coverage_axis,
        times,
        values["coverage"],
        values["coverage_history_ci"],
        labels,
        line_width,
        band_alpha,
    )
    _plot_histories(
        cost_axis,
        times,
        values["locational_cost"],
        values["locational_cost_history_ci"],
        labels,
        line_width,
        band_alpha,
    )
    coverage_axis.set_title(r"$\mathcal{E}$", fontsize=title_font_size)
    cost_axis.set_title(r"$\mathcal{L}$", fontsize=title_font_size)
    for axis in (coverage_axis, cost_axis):
        axis.set_xlabel("mission time [s]", fontsize=axis_label_font_size)
        axis.tick_params(axis="both", labelsize=tick_label_font_size)
        axis.grid(alpha=0.25)
    handles, legend_labels = coverage_axis.get_legend_handles_labels()
    figure.legend(
        handles,
        legend_labels,
        loc="outside lower center",
        ncol=2,
        frameon=False,
        fontsize=legend_font_size,
    )
    figure.savefig(output, dpi=dpi, facecolor="white")
    plt.close(figure)
    return output


def _plot_histories(
    axis,
    times: np.ndarray,
    histories: np.ndarray,
    confidence_intervals: np.ndarray,
    labels: dict[str, str],
    line_width: float = DEFAULT_LINE_WIDTH,
    band_alpha: float = DEFAULT_BAND_ALPHA,
) -> None:
    for index, method in enumerate(METHODS):
        color = COLORS[method]
        lower = confidence_intervals[index, :, 0]
        upper = confidence_intervals[index, :, 1]
        axis.fill_between(
            times, lower, upper, color=color, alpha=band_alpha, linewidth=0.0
        )
        axis.plot(
            times,
            np.mean(histories[index], axis=0),
            color=color,
            linestyle=LINESTYLES[method],
            linewidth=line_width,
            label=labels[method],
        )


def _labels(effective_ranges: np.ndarray) -> dict[str, str]:
    values = np.asarray(effective_ranges, dtype=float)
    if values.shape != (len(METHODS),):
        raise ValueError("effective aerial ranges must match the four methods")
    labels = dict(LABELS)
    labels["multifidelity_lr"] += rf" ($R_A={values[0]:g}$)"
    labels["egerstedt_lr"] += rf" ($R_A={values[1]:g}$)"
    return labels


def main() -> None:
    arguments = parse_args()
    output = plot_four_method_comparison(
        arguments.evaluated,
        arguments.output,
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
    print(f"four_method_metrics_plot={output}")


if __name__ == "__main__":
    main()
