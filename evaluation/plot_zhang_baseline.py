"""Plot trajectories and sweep summaries from a saved Zhang baseline run."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.zhang import power_diagram_labels  # noqa: E402
from evaluation.zhang_io import (  # noqa: E402
    ZHANG_BASELINE_RAW_ARCHIVE_KIND,
    load_archive,
)


DEFAULT_INPUT = Path("output/zhang_baseline/raw.npz")
DEFAULT_OUTPUT = Path("output/zhang_baseline/summary.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", "-i", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", "-o", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--condition",
        default=None,
        help="condition shown in the trajectory panel; defaults to the first",
    )
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def plot_zhang_baseline(
    input_path: str | Path,
    output_path: str | Path,
    *,
    condition: str | None = None,
    episode: int = 0,
    dpi: int = 180,
) -> Path:
    """Render one trajectory and aggregate cost/travel sweep summaries."""
    metadata, values = load_archive(
        input_path, expected_kind=ZHANG_BASELINE_RAW_ARCHIVE_KIND
    )
    names = [str(value) for value in values["condition_names"].tolist()]
    if not names:
        raise ValueError("Zhang archive contains no conditions")
    selected_name = names[0] if condition is None else condition
    if selected_name not in names:
        raise ValueError(
            f"unknown condition {selected_name!r}; available: {', '.join(names)}"
        )
    selected = names.index(selected_name)
    episode_count = int(metadata["episodes"])
    if (
        isinstance(episode, bool)
        or not isinstance(episode, int)
        or not (0 <= episode < episode_count)
    ):
        raise IndexError(f"episode must be between 0 and {episode_count - 1}")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")

    points = values["query_points"]
    rows, columns = (int(value) for value in values["grid_shape"])
    x_grid = points[:, 0].reshape(rows, columns)
    y_grid = points[:, 1].reshape(rows, columns)
    high_field = values["high_field"].reshape(rows, columns)
    aerial_count = int(values["aerial_counts"][selected])
    ground_count = int(values["ground_counts"][selected])
    aerial = values["aerial_positions"][selected, episode, :, :aerial_count]
    ground = values["ground_positions"][selected, episode, :, :ground_count]
    weights = values["cell_weights"][selected, episode, -1, :aerial_count]
    final_labels = power_diagram_labels(points, aerial[-1], weights).reshape(
        rows, columns
    )
    times = values["state_times"]

    figure, axes = plt.subplots(2, 2, figsize=(13.0, 10.0))
    trajectory_axis, history_axis, final_axis, travel_axis = axes.ravel()
    trajectory_axis.contourf(
        x_grid, y_grid, high_field, levels=18, cmap="magma", alpha=0.7
    )
    trajectory_axis.contour(
        x_grid,
        y_grid,
        final_labels,
        levels=np.arange(aerial_count + 1) - 0.5,
        colors="white",
        linewidths=0.7,
        alpha=0.8,
    )
    for index in range(ground_count):
        trajectory_axis.plot(
            ground[:, index, 0], ground[:, index, 1], color="tab:green", alpha=0.65
        )
        trajectory_axis.scatter(
            ground[-1, index, 0], ground[-1, index, 1], color="tab:green", s=18
        )
    for index in range(aerial_count):
        trajectory_axis.plot(
            aerial[:, index, 0],
            aerial[:, index, 1],
            linestyle="--",
            color="tab:cyan",
            alpha=0.85,
        )
        trajectory_axis.scatter(
            aerial[-1, index, 0],
            aerial[-1, index, 1],
            marker="^",
            color="tab:cyan",
            edgecolor="black",
            linewidth=0.4,
            s=38,
        )
    trajectory_axis.set_title(f"{selected_name}, episode {episode}: trajectories")
    trajectory_axis.set_xlabel("x")
    trajectory_axis.set_ylabel("y")
    trajectory_axis.set_aspect("equal")

    costs = values["locational_cost"]
    for index, name in enumerate(names):
        mean = np.mean(costs[index], axis=0)
        history_axis.plot(times, mean, label=name)
        if costs.shape[1] > 1:
            history_axis.fill_between(
                times,
                np.percentile(costs[index], 25.0, axis=0),
                np.percentile(costs[index], 75.0, axis=0),
                alpha=0.12,
            )
    history_axis.set_title("Locational coverage cost")
    history_axis.set_xlabel("Algorithm 2 time [s]")
    history_axis.set_ylabel(r"$H$")
    history_axis.grid(alpha=0.25)
    history_axis.legend(fontsize=7)

    compositions = list(
        dict.fromkeys(
            (int(aerial), int(ground))
            for aerial, ground in zip(
                values["aerial_counts"], values["ground_counts"], strict=True
            )
        )
    )
    sensing = values["ground_sensing_ranges"]
    for aerial_count_value, ground_count_value in compositions:
        selected_conditions = np.flatnonzero(
            (values["aerial_counts"] == aerial_count_value)
            & (values["ground_counts"] == ground_count_value)
        )
        order = selected_conditions[np.argsort(sensing[selected_conditions])]
        label = f"A{aerial_count_value}/G{ground_count_value}"
        final_axis.plot(
            sensing[order],
            np.mean(costs[order, :, -1], axis=1),
            marker="o",
            label=label,
        )
        travel_axis.plot(
            sensing[order],
            np.mean(values["ground_travel_distance"][order, :, -1], axis=1),
            marker="o",
            label=label,
        )
    final_axis.set_title("Final cost versus ground sensing range")
    final_axis.set_xlabel(r"$R_G$")
    final_axis.set_ylabel(r"final $H$")
    final_axis.grid(alpha=0.25)
    final_axis.legend(fontsize=8)
    travel_axis.set_title("Ground travel versus sensing range")
    travel_axis.set_xlabel(r"$R_G$")
    travel_axis.set_ylabel("accumulated distance")
    travel_axis.grid(alpha=0.25)
    travel_axis.legend(fontsize=8)

    figure.suptitle(
        "Standalone distributed Zhang air-ground coverage baseline\n"
        "obstacle-free domain; curves show episode means",
        fontsize=13,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=dpi, bbox_inches="tight", pad_inches=0.18)
    plt.close(figure)
    return destination


def main() -> None:
    arguments = parse_args()
    path = plot_zhang_baseline(
        arguments.input,
        arguments.output,
        condition=arguments.condition,
        episode=arguments.episode,
        dpi=arguments.dpi,
    )
    print(f"zhang_baseline_plot={path}")


if __name__ == "__main__":
    main()
