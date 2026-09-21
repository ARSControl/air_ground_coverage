"""Plot one predeclared episode from the four-condition comparison."""

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
from matplotlib.colors import Normalize  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
import numpy as np  # noqa: E402

from evaluation.comparison_io import (  # noqa: E402
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    SCENARIO_ARRAYS,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    scenario_fingerprint,
)
from evaluation.plot_multifidelity_composition import (  # noqa: E402
    DEFAULT_AXIS_LABEL_FONT_SIZE,
    DEFAULT_TICK_LABEL_FONT_SIZE,
    DEFAULT_TITLE_FONT_SIZE,
)


DEFAULT_FIGURE_WIDTH = 3.5
DEFAULT_FIGURE_HEIGHT = 4.0
DEFAULT_DPI = 300
TRUTH_COLORMAP = "Greys"

# Paper colors. Edit these constants to restyle every trajectory panel.
AERIAL_COLOR = "tab:orange"
GROUND_COLOR = "tab:green"
AERIAL_FOOTPRINT_COLOR = AERIAL_COLOR
GROUND_FOOTPRINT_COLOR = "#707070"
ENDPOINT_EDGE_COLOR = "black"
FIGURE_BACKGROUND_COLOR = "white"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("output/four_method_comparison"),
        help="four-condition pipeline output directory",
    )
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--output", "-o", type=Path, default=None)
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
    style.add_argument("--dpi", type=int, default=DEFAULT_DPI)
    return parser.parse_args()


def plot_four_method_trajectories(
    input_directory: str | Path,
    output_path: str | Path,
    *,
    episode: int,
    figure_width: float = DEFAULT_FIGURE_WIDTH,
    figure_height: float = DEFAULT_FIGURE_HEIGHT,
    title_font_size: float = DEFAULT_TITLE_FONT_SIZE,
    axis_label_font_size: float = DEFAULT_AXIS_LABEL_FONT_SIZE,
    tick_label_font_size: float = DEFAULT_TICK_LABEL_FONT_SIZE,
    dpi: int = DEFAULT_DPI,
) -> Path:
    """Render a common-truth 2x2 trajectory diagnostic for one episode."""
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    root = Path(input_directory)
    raw_metadata: list[dict[str, object]] = []
    raw: list[dict[str, np.ndarray]] = []
    for relative, kind in (
        ("multifidelity/raw.npz", MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND),
        ("egerstedt_lr/raw.npz", EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND),
        ("egerstedt_full/raw.npz", EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND),
        ("zhang/raw.npz", ZHANG_COMPARISON_RAW_ARCHIVE_KIND),
    ):
        metadata, values = load_archive(root / relative, expected_kind=kind)
        raw_metadata.append(metadata)
        raw.append(values)
    _validate(raw_metadata, raw, episode)
    points = raw[0]["query_points"]
    x_values = np.unique(points[:, 0])
    y_values = np.unique(points[:, 1])
    truth = raw[0]["truth_density"][episode].reshape(y_values.size, x_values.size)
    extent = (x_values[0], x_values[-1], y_values[0], y_values[-1])
    normalization = Normalize(vmin=0.0, vmax=float(np.max(truth)))
    aerial = (
        raw[0]["aerial_states"][episode, :, :, :2],
        raw[1]["aerial_positions"][episode],
        raw[2]["aerial_positions"][episode],
        raw[3]["aerial_positions"][episode],
    )
    ground = (
        raw[0]["ground_states"][episode, :, :, :2],
        raw[1]["ground_positions"][episode],
        raw[2]["ground_positions"][episode],
        raw[3]["ground_positions"][episode],
    )
    aerial_ranges = (
        _scalar(raw[0]["aerial_sensing_range"]),
        _scalar(raw[1]["effective_aerial_sensing_range"]),
        _scalar(raw[2]["effective_aerial_sensing_range"]),
        np.inf,
    )
    titles = (
        "Proposed",
        "Rudolph [17] - LR",
        "Rudolph [17]",
        "Zhang [18]",
    )
    figure, axes = plt.subplots(
        2,
        2,
        figsize=(figure_width, figure_height),
        constrained_layout=True,
    )
    for index, axis in enumerate(axes.ravel()):
        axis.imshow(
            truth,
            origin="lower",
            extent=extent,
            cmap=TRUTH_COLORMAP,
            norm=normalization,
            interpolation="nearest",
            aspect="equal",
        )
        _plot_team(axis, aerial[index], ground[index])
        if np.isfinite(aerial_ranges[index]) and aerial_ranges[index] < max(
            extent[1] - extent[0], extent[3] - extent[2]
        ):
            for final_position in aerial[index][-1]:
                axis.add_patch(
                    Circle(
                        final_position,
                        radius=aerial_ranges[index],
                        fill=False,
                        edgecolor=AERIAL_FOOTPRINT_COLOR,
                        linestyle="--",
                        linewidth=0.7,
                        alpha=0.55,
                    )
                )
        ground_range = _scalar(raw[index]["ground_sensing_range"])
        for final_position in ground[index][-1]:
            axis.add_patch(
                Circle(
                    final_position,
                    radius=ground_range,
                    fill=False,
                    edgecolor=GROUND_FOOTPRINT_COLOR,
                    linewidth=0.65,
                    alpha=0.8,
                )
            )
        axis.set_title(titles[index], fontsize=title_font_size)
        axis.set_xlabel("x", fontsize=axis_label_font_size)
        axis.set_ylabel("y", fontsize=axis_label_font_size)
        axis.tick_params(axis="both", labelsize=tick_label_font_size)
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi, facecolor=FIGURE_BACKGROUND_COLOR)
    plt.close(figure)
    return output


def _plot_team(axis, aerial: np.ndarray, ground: np.ndarray) -> None:
    for robot in range(aerial.shape[1]):
        axis.plot(
            aerial[:, robot, 0],
            aerial[:, robot, 1],
            linestyle="--",
            color=AERIAL_COLOR,
            linewidth=1.1,
            alpha=0.85,
        )
    for robot in range(ground.shape[1]):
        axis.plot(
            ground[:, robot, 0],
            ground[:, robot, 1],
            color=GROUND_COLOR,
            linewidth=1.2,
            alpha=0.82,
        )
    axis.scatter(aerial[0, :, 0], aerial[0, :, 1], marker="x", color=AERIAL_COLOR)
    axis.scatter(ground[0, :, 0], ground[0, :, 1], marker="x", color=GROUND_COLOR)
    axis.scatter(
        aerial[-1, :, 0],
        aerial[-1, :, 1],
        marker="^",
        color=AERIAL_COLOR,
        edgecolor=ENDPOINT_EDGE_COLOR,
        linewidth=0.4,
    )
    axis.scatter(
        ground[-1, :, 0],
        ground[-1, :, 1],
        marker="o",
        color=GROUND_COLOR,
        edgecolor=ENDPOINT_EDGE_COLOR,
        linewidth=0.4,
    )


def _validate(
    metadata: list[dict[str, object]],
    raw: list[dict[str, np.ndarray]],
    episode: int,
) -> str:
    fingerprints = [scenario_fingerprint(values) for values in raw]
    recorded = [values.get("scenario_fingerprint") for values in metadata]
    if len(set((*fingerprints, *recorded))) != 1:
        raise ValueError("trajectory inputs do not share one scenario fingerprint")
    for name in SCENARIO_ARRAYS:
        if not all(np.array_equal(raw[0][name], values[name]) for values in raw[1:]):
            raise ValueError(f"trajectory inputs differ in scenario array {name!r}")
    if episode >= raw[0]["seeds"].size:
        raise IndexError("episode is outside the paired archives")
    limited = metadata[1].get("aerial_sensing")
    full = metadata[2].get("aerial_sensing")
    zhang = metadata[3].get("aerial_sensing")
    if not isinstance(limited, dict) or limited.get("overridden") is not False:
        raise ValueError("second panel must contain Egerstedt-LR")
    if not isinstance(full, dict) or full.get("covers_complete_domain") is not True:
        raise ValueError("third panel must contain full-range Egerstedt")
    if not isinstance(zhang, dict) or zhang.get("mode") != "full_cell":
        raise ValueError("fourth panel must contain standard full-cell Zhang")
    return fingerprints[0]


def _scalar(value: np.ndarray) -> float:
    array = np.asarray(value, dtype=float)
    if array.size != 1 or not np.isfinite(array.reshape(-1)[0]):
        raise ValueError("expected a finite scalar archive value")
    return float(array.reshape(-1)[0])


def main() -> None:
    arguments = parse_args()
    output = (
        arguments.output
        if arguments.output is not None
        else arguments.input_dir
        / "plots"
        / f"trajectories_episode_{arguments.episode:03d}.png"
    )
    result = plot_four_method_trajectories(
        arguments.input_dir,
        output,
        episode=arguments.episode,
        figure_width=arguments.figure_width,
        figure_height=arguments.figure_height,
        title_font_size=arguments.title_font_size,
        axis_label_font_size=arguments.axis_label_font_size,
        tick_label_font_size=arguments.tick_label_font_size,
        dpi=arguments.dpi,
    )
    print(f"four_method_trajectory_plot={result}")


if __name__ == "__main__":
    main()
