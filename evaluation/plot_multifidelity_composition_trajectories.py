"""Plot one explicitly selected episode from a raw composition archive."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    RAW_ARCHIVE_KIND,
    load_archive,
)
from evaluation.multifidelity_metrics import clipped_reconstruction  # noqa: E402


DEFAULT_INPUT = Path("output/multifidelity_composition_raw.npz")
DEFAULT_OUTPUT_DIRECTORY = Path("output/multifidelity_composition_trajectories")
AERIAL_COLOR = "#2563EB"
GROUND_COLOR = "#EA580C"


@dataclass(frozen=True)
class FinalPosteriorRecord:
    """Latest latent HIGH-posterior record saved for one episode."""

    high_mean: np.ndarray
    step: int
    time: float
    version: int
    low_kernel_length_scale: float | None = None
    discrepancy_kernel_length_scale: float | None = None
    hyperparameter_fit_performed: bool | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", "-i", type=Path, default=DEFAULT_INPUT)
    parser.add_argument(
        "--composition",
        "-c",
        required=True,
        help="exact saved label, for example A2/G6",
    )
    parser.add_argument(
        "--episode",
        "-e",
        type=int,
        default=0,
        help="zero-based saved episode index (default: 0)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="PNG destination; defaults to an explicit composition/episode name",
    )
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def plot_composition_trajectories(
    raw_path: str | Path,
    *,
    composition: str,
    episode: int,
    output_path: str | Path | None = None,
    dpi: int = 180,
) -> Path:
    """Render saved trajectories without rerunning or mutating the simulation."""
    if not isinstance(composition, str) or not composition.strip():
        raise TypeError("composition must be a nonempty string")
    if isinstance(episode, bool) or not isinstance(episode, int):
        raise TypeError("episode must be an integer")
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")

    metadata, values = load_archive(raw_path, expected_kind=RAW_ARCHIVE_KIND)
    names = tuple(str(value) for value in values["composition_names"].tolist())
    requested = composition.strip()
    if requested not in names:
        choices = ", ".join(names)
        raise ValueError(
            f"unknown composition {requested!r}; available compositions: {choices}"
        )
    composition_index = names.index(requested)
    episode_count = int(values["seeds"].size)
    if episode < 0 or episode >= episode_count:
        raise IndexError(f"episode index {episode} is outside [0, {episode_count - 1}]")

    aerial_count = int(values["aerial_counts"][composition_index])
    ground_count = int(values["ground_counts"][composition_index])
    aerial_states = _selected_states(
        values, "aerial_states", composition_index, episode, aerial_count
    )
    ground_states = _selected_states(
        values, "ground_states", composition_index, episode, ground_count
    )
    map_grid = np.asarray(values["map_grid"][composition_index, episode])
    if map_grid.ndim != 2 or not np.all(np.isin(map_grid, (0, 1))):
        raise ValueError("saved map_grid episode must be a two-dimensional binary map")

    query_shape = tuple(int(value) for value in metadata["query_shape"])
    query_points = np.asarray(values["query_points"], dtype=float)
    truth = np.asarray(values["high_truth"][composition_index, episode], dtype=float)
    if len(query_shape) != 2 or np.prod(query_shape) != truth.size:
        raise ValueError("saved query_shape is incompatible with high_truth")
    if query_points.shape != (truth.size, 2):
        raise ValueError("saved query_points are incompatible with high_truth")
    grid_x = query_points[:, 0].reshape(query_shape)
    grid_y = query_points[:, 1].reshape(query_shape)
    truth_grid = truth.reshape(query_shape)
    free_mask = np.asarray(values["free_mask"][composition_index, episode], dtype=bool)
    if free_mask.shape != truth.shape:
        raise ValueError("saved free_mask is incompatible with high_truth")
    posterior = select_final_posterior_record(values, composition_index, episode)
    if posterior.high_mean.shape != truth.shape:
        raise ValueError("saved posterior_mean is incompatible with high_truth")
    reconstructed = clipped_reconstruction(posterior.high_mean)
    reconstructed_grid = reconstructed.reshape(query_shape)
    free_grid = free_mask.reshape(query_shape)
    truth_visible = np.ma.masked_where(~free_grid, truth_grid)
    reconstruction_visible = np.ma.masked_where(~free_grid, reconstructed_grid)
    error_visible = np.ma.masked_where(
        ~free_grid, np.abs(reconstructed_grid - truth_grid)
    )
    comparison_values = np.concatenate((truth[free_mask], reconstructed[free_mask]))
    field_minimum, field_maximum = _finite_limits(comparison_values)
    _, error_maximum = _finite_limits(error_visible.compressed(), include_zero=True)

    figure, axes = plt.subplots(2, 2, figsize=(13.2, 10.8), constrained_layout=True)
    trajectory_axis, truth_axis, reconstruction_axis, error_axis = axes.ravel()
    _plot_field(
        trajectory_axis,
        grid_x,
        grid_y,
        truth_visible,
        vmin=field_minimum,
        vmax=field_maximum,
    )
    _plot_obstacles(trajectory_axis, map_grid)
    _plot_team(trajectory_axis, aerial_states, AERIAL_COLOR)
    _plot_team(trajectory_axis, ground_states, GROUND_COLOR)
    trajectory_axis.set_title("A. Saved closed-loop trajectories")
    trajectory_axis.legend(
        handles=_legend_handles(aerial_count, ground_count, bool(np.any(map_grid))),
        loc="upper right",
        framealpha=0.88,
    )

    truth_plot = _plot_field(
        truth_axis,
        grid_x,
        grid_y,
        truth_visible,
        vmin=field_minimum,
        vmax=field_maximum,
    )
    _plot_obstacles(truth_axis, map_grid)
    truth_axis.set_title("B. Hidden HIGH-fidelity ground truth")
    figure.colorbar(truth_plot, ax=truth_axis, label="Latent HIGH field")

    reconstruction_plot = _plot_field(
        reconstruction_axis,
        grid_x,
        grid_y,
        reconstruction_visible,
        vmin=field_minimum,
        vmax=field_maximum,
    )
    _plot_obstacles(reconstruction_axis, map_grid)
    reconstruction_axis.set_title(
        "C. Final zero-clipped HIGH reconstruction\n"
        f"step={posterior.step}, t={posterior.time:g} s, version={posterior.version}"
        f"{_kernel_title(posterior)}"
    )
    figure.colorbar(
        reconstruction_plot, ax=reconstruction_axis, label="Latent HIGH field"
    )

    error_plot = _plot_field(
        error_axis,
        grid_x,
        grid_y,
        error_visible,
        cmap="magma",
        vmin=0.0,
        vmax=error_maximum,
    )
    _plot_obstacles(error_axis, map_grid)
    error_axis.set_title("D. Absolute zero-clipped reconstruction error")
    figure.colorbar(error_plot, ax=error_axis, label="Absolute error")

    height, width = map_grid.shape
    seed = int(values["seeds"][episode])
    scenario = str(metadata.get("scenario", "default"))
    time_values = np.asarray(values["state_times"], dtype=float)
    mission_end = float(time_values[-1]) if time_values.size else 0.0
    figure.suptitle(
        f"{scenario}: {requested}, episode index {episode}, seed {seed}: "
        f"saved mission through t={mission_end:g} s",
        fontsize=15,
    )
    for axis in axes.ravel():
        axis.set_xlabel("x [map units]")
        axis.set_ylabel("y [map units]")
        axis.set_xlim(0.0, float(width))
        axis.set_ylim(0.0, float(height))
        axis.set_aspect("equal")
        axis.grid(color="white", alpha=0.12, linewidth=0.6)

    destination = (
        _default_output(requested, episode)
        if output_path is None
        else Path(output_path)
    )
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def select_final_posterior_record(
    values: dict[str, np.ndarray], composition_index: int, episode: int
) -> FinalPosteriorRecord:
    """Select the last saved posterior by step, timestamp, then version."""
    compositions = np.asarray(values["posterior_composition"], dtype=int)
    episodes = np.asarray(values["posterior_episode"], dtype=int)
    selected = np.flatnonzero(
        (compositions == composition_index) & (episodes == episode)
    )
    if selected.size == 0:
        raise RuntimeError(
            "selected composition episode contains no saved posterior record"
        )
    steps = np.asarray(values["posterior_step"], dtype=int)
    times = np.asarray(values["posterior_time"], dtype=float)
    versions = np.asarray(values["posterior_version"], dtype=int)
    index = int(
        selected[np.lexsort((versions[selected], times[selected], steps[selected]))[-1]]
    )
    high_mean = np.array(values["posterior_mean"][index], dtype=float, copy=True)
    if high_mean.ndim != 1 or not np.all(np.isfinite(high_mean)):
        raise ValueError("selected posterior_mean must be a finite vector")
    high_mean.setflags(write=False)
    return FinalPosteriorRecord(
        high_mean=high_mean,
        step=int(steps[index]),
        time=float(times[index]),
        version=int(versions[index]),
        low_kernel_length_scale=_optional_positive_record(
            values, "posterior_low_kernel_length_scale", index
        ),
        discrepancy_kernel_length_scale=_optional_positive_record(
            values, "posterior_discrepancy_kernel_length_scale", index
        ),
        hyperparameter_fit_performed=(
            None
            if "posterior_hyperparameter_fit_performed" not in values
            else bool(values["posterior_hyperparameter_fit_performed"][index])
        ),
    )


def _optional_positive_record(
    values: dict[str, np.ndarray], name: str, index: int
) -> float | None:
    if name not in values:
        return None
    value = float(values[name][index])
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must contain finite positive values")
    return value


def _kernel_title(posterior: FinalPosteriorRecord) -> str:
    if (
        posterior.low_kernel_length_scale is None
        or posterior.discrepancy_kernel_length_scale is None
    ):
        return ""
    status = "fitted" if posterior.hyperparameter_fit_performed else "reused"
    return (
        "\n"
        f"kernels {status}: LOW length={posterior.low_kernel_length_scale:.3g}, "
        "discrepancy length="
        f"{posterior.discrepancy_kernel_length_scale:.3g}"
    )


def _selected_states(
    values: dict[str, np.ndarray],
    key: str,
    composition_index: int,
    episode: int,
    robot_count: int,
) -> np.ndarray:
    states = np.asarray(values[key][composition_index, episode], dtype=float)
    if states.ndim != 3 or states.shape[2] < 2 or states.shape[1] < robot_count:
        raise ValueError(f"saved {key} has an incompatible shape")
    selected = states[:, :robot_count, :2]
    if not np.all(np.isfinite(selected)):
        raise ValueError(f"active saved {key} entries must be finite")
    return selected


def _plot_field(
    axis,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    field: np.ndarray,
    *,
    cmap: str = "viridis",
    vmin: float,
    vmax: float,
):
    return axis.pcolormesh(
        grid_x,
        grid_y,
        field,
        shading="auto",
        cmap=cmap,
        vmin=vmin,
        vmax=vmax,
        rasterized=True,
    )


def _plot_obstacles(axis, map_grid: np.ndarray) -> None:
    obstacle_overlay = np.ma.masked_where(map_grid == 0, map_grid)
    height, width = map_grid.shape
    axis.imshow(
        obstacle_overlay,
        origin="lower",
        extent=(0.0, float(width), 0.0, float(height)),
        cmap=ListedColormap(["#202020"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        alpha=0.78,
        zorder=2,
    )


def _finite_limits(
    values: np.ndarray, *, include_zero: bool = False
) -> tuple[float, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        raise ValueError("plotted field contains no finite free-space values")
    minimum = min(0.0, float(np.min(finite))) if include_zero else float(np.min(finite))
    maximum = float(np.max(finite))
    if np.isclose(minimum, maximum):
        padding = max(abs(minimum), 1.0) * 1.0e-6
        minimum -= padding
        maximum += padding
    return minimum, maximum


def _plot_team(axis, states: np.ndarray, color: str) -> None:
    for robot_index in range(states.shape[1]):
        trajectory = states[:, robot_index]
        axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            color=color,
            linewidth=2.0,
            alpha=0.88,
            zorder=3,
        )
        axis.scatter(
            trajectory[0, 0],
            trajectory[0, 1],
            color=color,
            edgecolor="white",
            linewidth=0.7,
            marker="o",
            s=34,
            zorder=4,
        )
        axis.scatter(
            trajectory[-1, 0],
            trajectory[-1, 1],
            color=color,
            linewidth=1.8,
            marker="x",
            s=45,
            zorder=4,
        )


def _legend_handles(aerial_count: int, ground_count: int, has_obstacles: bool) -> list:
    handles: list = []
    if aerial_count:
        handles.append(Line2D([0], [0], color=AERIAL_COLOR, lw=2, label="Aerial"))
    if ground_count:
        handles.append(Line2D([0], [0], color=GROUND_COLOR, lw=2, label="Ground"))
    handles.extend(
        (
            Line2D(
                [0],
                [0],
                color="#404040",
                marker="o",
                linestyle="none",
                label="Start",
            ),
            Line2D(
                [0],
                [0],
                color="#404040",
                marker="x",
                linestyle="none",
                label="End",
            ),
        )
    )
    if has_obstacles:
        handles.append(Patch(facecolor="#202020", label="Ground obstacle"))
    return handles


def _default_output(composition: str, episode: int) -> Path:
    safe_composition = re.sub(r"[^A-Za-z0-9_-]+", "_", composition).strip("_")
    return DEFAULT_OUTPUT_DIRECTORY / (f"{safe_composition}_episode_{episode:03d}.png")


def main() -> None:
    arguments = parse_args()
    destination = plot_composition_trajectories(
        arguments.input,
        composition=arguments.composition,
        episode=arguments.episode,
        output_path=arguments.output,
        dpi=arguments.dpi,
    )
    print(f"composition_trajectory_plot={destination}")


if __name__ == "__main__":
    main()
