"""Create separate paired trajectory plots for proposed and baseline methods."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import Normalize  # noqa: E402
from matplotlib.patches import Circle, Wedge  # noqa: E402
import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    scenario_fingerprint,
)


METHOD_LABELS = {
    "multifidelity": "Proposed multi-fidelity method",
    "egerstedt": "Egerstedt baseline",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--multifidelity",
        type=Path,
        default=Path("output/baseline_comparison/multifidelity/raw.npz"),
    )
    parser.add_argument(
        "--egerstedt",
        type=Path,
        default=Path("output/baseline_comparison/egerstedt/raw.npz"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("output/baseline_comparison"),
    )
    parser.add_argument("--episode", type=int, default=0)
    return parser.parse_args()


def plot_baseline_comparison(
    multifidelity_path: str | Path,
    egerstedt_path: str | Path,
    *,
    output_root: str | Path,
    episode: int = 0,
) -> tuple[Path, Path]:
    """Write one plot per method under separate result subdirectories."""
    multifidelity_metadata, multifidelity = load_archive(
        multifidelity_path,
        expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    )
    egerstedt_metadata, egerstedt = load_archive(
        egerstedt_path,
        expected_kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    )
    _validate_pair(
        multifidelity_metadata,
        multifidelity,
        egerstedt_metadata,
        egerstedt,
        episode,
    )

    points = multifidelity["query_points"]
    x_values, y_values, truth = _structured_truth(
        points, multifidelity["truth_density"][episode]
    )
    extent = (
        float(x_values[0]),
        float(x_values[-1]),
        float(y_values[0]),
        float(y_values[-1]),
    )
    normalization = Normalize(vmin=0.0, vmax=float(np.max(truth)))
    obstacle_map = multifidelity["map_grid"][episode].astype(bool)
    seed = int(multifidelity["seeds"][episode])
    mission_time = float(multifidelity["state_times"][-1])
    root = Path(output_root)
    filename = f"trajectories_episode_{episode:03d}.png"
    multifidelity_output = root / "multifidelity" / filename
    egerstedt_output = root / "egerstedt" / filename

    _render_method_plot(
        output_path=multifidelity_output,
        method="multifidelity",
        truth=truth,
        extent=extent,
        normalization=normalization,
        obstacle_map=obstacle_map,
        aerial_positions=multifidelity["aerial_states"][episode, :, :, :2],
        ground_positions=multifidelity["ground_states"][episode, :, :, :2],
        ground_headings=multifidelity["ground_states"][episode, -1, :, 2],
        ground_fov_degrees=float(multifidelity["ground_fov_degrees"]),
        ground_sensing_range=float(multifidelity["ground_sensing_range"]),
        seed=seed,
        mission_time=mission_time,
    )
    _render_method_plot(
        output_path=egerstedt_output,
        method="egerstedt",
        truth=truth,
        extent=extent,
        normalization=normalization,
        obstacle_map=obstacle_map,
        aerial_positions=egerstedt["aerial_positions"][episode],
        ground_positions=egerstedt["ground_positions"][episode],
        ground_headings=np.zeros(egerstedt["ground_positions"].shape[2]),
        ground_fov_degrees=float(egerstedt["ground_fov_degrees"]),
        ground_sensing_range=float(egerstedt["ground_sensing_range"]),
        seed=seed,
        mission_time=mission_time,
    )
    return multifidelity_output, egerstedt_output


def _validate_pair(
    multifidelity_metadata: dict[str, object],
    multifidelity: dict[str, np.ndarray],
    egerstedt_metadata: dict[str, object],
    egerstedt: dict[str, np.ndarray],
    episode: int,
) -> None:
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    multifidelity_fingerprint = scenario_fingerprint(multifidelity)
    egerstedt_fingerprint = scenario_fingerprint(egerstedt)
    if not (
        multifidelity_metadata.get("scenario_fingerprint")
        == egerstedt_metadata.get("scenario_fingerprint")
        == multifidelity_fingerprint
        == egerstedt_fingerprint
    ):
        raise ValueError("method archives do not contain the same paired scenario")
    episode_count = multifidelity["seeds"].size
    if episode >= episode_count:
        raise IndexError(
            f"episode {episode} is out of range for {episode_count} episodes"
        )
    if not np.array_equal(multifidelity["truth_density"], egerstedt["truth_density"]):
        raise ValueError("method archives do not contain identical truth densities")


def _structured_truth(
    query_points: np.ndarray, density: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = np.asarray(query_points, dtype=float)
    values = np.asarray(density, dtype=float)
    x_values = np.unique(points[:, 0])
    y_values = np.unique(points[:, 1])
    if x_values.size * y_values.size != points.shape[0]:
        raise ValueError("trajectory plots require a Cartesian query grid")
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    expected = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    if not np.array_equal(points, expected):
        raise ValueError("query_points must use row-major Cartesian-grid ordering")
    if values.shape != (points.shape[0],):
        raise ValueError("truth density must match query_points")
    return x_values, y_values, values.reshape(y_values.size, x_values.size)


def _render_method_plot(
    *,
    output_path: Path,
    method: str,
    truth: np.ndarray,
    extent: tuple[float, float, float, float],
    normalization: Normalize,
    obstacle_map: np.ndarray,
    aerial_positions: np.ndarray,
    ground_positions: np.ndarray,
    ground_headings: np.ndarray,
    ground_fov_degrees: float,
    ground_sensing_range: float,
    seed: int,
    mission_time: float,
) -> None:
    figure, axis = plt.subplots(figsize=(7.2, 6.2), constrained_layout=True)
    image = axis.imshow(
        truth,
        origin="lower",
        extent=extent,
        cmap="magma",
        norm=normalization,
        aspect="equal",
        interpolation="nearest",
    )
    if np.any(obstacle_map):
        obstacle_overlay = np.ma.masked_where(~obstacle_map, obstacle_map)
        axis.imshow(
            obstacle_overlay,
            origin="lower",
            extent=extent,
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            alpha=0.75,
            interpolation="nearest",
        )

    _plot_ground_footprints(
        axis,
        ground_positions[-1],
        ground_headings,
        fov_degrees=ground_fov_degrees,
        sensing_range=ground_sensing_range,
    )
    for robot in range(aerial_positions.shape[1]):
        axis.plot(
            aerial_positions[:, robot, 0],
            aerial_positions[:, robot, 1],
            "--",
            color="#60a5fa",
            linewidth=2.0,
            label="Aerial trajectories" if robot == 0 else None,
        )
    for robot in range(ground_positions.shape[1]):
        axis.plot(
            ground_positions[:, robot, 0],
            ground_positions[:, robot, 1],
            color="#00e5ff",
            linewidth=2.6,
            label="Ground trajectories" if robot == 0 else None,
        )
    axis.scatter(
        aerial_positions[0, :, 0],
        aerial_positions[0, :, 1],
        marker="x",
        s=70,
        linewidths=2.0,
        color="white",
        label="Initial positions",
        zorder=6,
    )
    axis.scatter(
        ground_positions[0, :, 0],
        ground_positions[0, :, 1],
        marker="x",
        s=70,
        linewidths=2.0,
        color="white",
        zorder=6,
    )
    axis.scatter(
        aerial_positions[-1, :, 0],
        aerial_positions[-1, :, 1],
        marker="^",
        s=80,
        color="#60a5fa",
        edgecolor="white",
        linewidth=1.0,
        label="Final aerial positions",
        zorder=7,
    )
    axis.scatter(
        ground_positions[-1, :, 0],
        ground_positions[-1, :, 1],
        marker="o",
        s=70,
        color="#00e5ff",
        edgecolor="white",
        linewidth=1.0,
        label="Final ground positions",
        zorder=7,
    )
    padding = 0.015 * max(extent[1] - extent[0], extent[3] - extent[2])
    axis.set_xlim(extent[0] - padding, extent[1] + padding)
    axis.set_ylim(extent[2] - padding, extent[3] + padding)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(
        f"{METHOD_LABELS[method]}\nseed {seed}, mission time {mission_time:g} s"
    )
    axis.legend(loc="lower left", fontsize=8, framealpha=0.9)
    figure.colorbar(image, ax=axis, label="Hidden truth probability density")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor="white")
    plt.close(figure)


def _plot_ground_footprints(
    axis,
    final_positions: np.ndarray,
    final_headings: np.ndarray,
    *,
    fov_degrees: float,
    sensing_range: float,
) -> None:
    if final_headings.shape != (final_positions.shape[0],):
        raise ValueError("ground headings must match final ground positions")
    for position, heading in zip(final_positions, final_headings, strict=True):
        if fov_degrees >= 360.0:
            footprint = Circle(
                position,
                sensing_range,
                facecolor="#00e5ff",
                edgecolor="white",
                linewidth=1.3,
                linestyle="--",
                alpha=0.18,
            )
        else:
            heading_degrees = float(np.rad2deg(heading))
            footprint = Wedge(
                position,
                sensing_range,
                heading_degrees - 0.5 * fov_degrees,
                heading_degrees + 0.5 * fov_degrees,
                facecolor="#00e5ff",
                edgecolor="white",
                linewidth=1.3,
                linestyle="--",
                alpha=0.2,
            )
        axis.add_patch(footprint)


def main() -> None:
    arguments = parse_args()
    multifidelity_output, egerstedt_output = plot_baseline_comparison(
        arguments.multifidelity,
        arguments.egerstedt,
        output_root=arguments.output_root,
        episode=arguments.episode,
    )
    print(f"multifidelity_trajectory_plot={multifidelity_output}")
    print(f"egerstedt_trajectory_plot={egerstedt_output}")


if __name__ == "__main__":
    main()
