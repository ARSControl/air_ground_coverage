"""Render trajectories and the final multi-fidelity posterior from one run."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Wedge  # noqa: E402
import numpy as np

from src.core.multifidelity_estimator import PosteriorSnapshot
from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import (
    CoupledSimulationResult,
    build_coupled_simulation,
)


def render_final_multifidelity_state(
    result: CoupledSimulationResult,
    snapshot: PosteriorSnapshot,
    ground_truth: np.ndarray,
    output_path: str | Path,
    *,
    ground_fov_degrees: float | None = None,
    ground_sensing_range: float | None = None,
    ground_final_states: np.ndarray | None = None,
) -> Path:
    """Save a four-panel summary of the final posterior and trajectories."""
    grid_x, grid_y = _structured_grid(snapshot)
    high_mean = np.asarray(snapshot.high_mean).reshape(snapshot.query_shape)
    ground_importance = _ground_importance_grid(snapshot)
    high_std = np.sqrt(
        np.maximum(np.asarray(snapshot.high_variance), 0.0)
    ).reshape(snapshot.query_shape)
    truth_x, truth_y, truth = _truth_grid(ground_truth, grid_x, grid_y)
    field_limits = (
        float(min(np.min(high_mean), np.min(truth))),
        float(max(np.max(high_mean), np.max(truth))),
    )
    if field_limits[0] == field_limits[1]:
        field_limits = (field_limits[0] - 0.5, field_limits[1] + 0.5)

    figure, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    image = axes[0, 0].pcolormesh(
        grid_x,
        grid_y,
        ground_importance,
        shading="auto",
        cmap="magma",
    )
    axes[0, 0].set_title("A. Controller density: normalized positive GP mean")
    figure.colorbar(
        image, ax=axes[0, 0], label="normalized importance density"
    )

    image = axes[0, 1].pcolormesh(
        grid_x, grid_y, high_std, shading="auto", cmap="viridis"
    )
    axes[0, 1].set_title("B. Final high-fidelity GP standard deviation")
    figure.colorbar(image, ax=axes[0, 1], label="standard deviation")

    image = axes[1, 0].pcolormesh(
        grid_x,
        grid_y,
        high_mean,
        shading="auto",
        cmap="magma",
        vmin=field_limits[0],
        vmax=field_limits[1],
    )
    _plot_combined_trajectories(
        axes[1, 0],
        result.aerial_trajectories,
        result.ground_trajectories,
        ground_fov_degrees=ground_fov_degrees,
        ground_sensing_range=ground_sensing_range,
        ground_final_states=ground_final_states,
    )
    axes[1, 0].set_title("C. All robot trajectories over final GP estimate")
    figure.colorbar(image, ax=axes[1, 0], label="estimated field value")

    image = axes[1, 1].pcolormesh(
        truth_x,
        truth_y,
        truth,
        shading="auto",
        cmap="magma",
        vmin=field_limits[0],
        vmax=field_limits[1],
    )
    _plot_combined_trajectories(
        axes[1, 1],
        result.aerial_trajectories,
        result.ground_trajectories,
        ground_fov_degrees=ground_fov_degrees,
        ground_sensing_range=ground_sensing_range,
        ground_final_states=ground_final_states,
    )
    axes[1, 1].set_title("D. All robot trajectories over simulator ground truth")
    figure.colorbar(image, ax=axes[1, 1], label="ground-truth field value")

    x_limits = (float(np.min(grid_x)), float(np.max(grid_x)))
    y_limits = (float(np.min(grid_y)), float(np.max(grid_y)))
    for axis in axes.flat:
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_xlim(*x_limits)
        axis.set_ylim(*y_limits)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, color="white", alpha=0.18, linewidth=0.7)

    hyperparameter_text = ""
    if hasattr(snapshot, "low_kernel_length_scale"):
        fit_status = (
            "fitted this update"
            if snapshot.hyperparameter_fit_performed
            else "reused"
        )
        hyperparameter_text = (
            "\n"
            f"kernels ({fit_status}): LOW length="
            f"{snapshot.low_kernel_length_scale:.3g}, discrepancy length="
            f"{snapshot.discrepancy_kernel_length_scale:.3g}"
        )
    figure.suptitle(
        "Final multi-fidelity simulation state\n"
        f"posterior v{snapshot.version}; "
        f"LOW samples={snapshot.low_sample_count}, "
        f"HIGH samples={snapshot.high_sample_count}"
        f"{hyperparameter_text}",
        fontsize=14,
    )
    path = Path(output_path)
    if not path.name:
        raise ValueError("output_path must name a file")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return path


def _structured_grid(snapshot: PosteriorSnapshot) -> tuple[np.ndarray, np.ndarray]:
    if len(snapshot.query_shape) != 2:
        raise ValueError("final posterior plot requires a two-dimensional grid")
    points = np.asarray(snapshot.query_points, dtype=float)
    expected_count = int(np.prod(snapshot.query_shape))
    if points.shape != (expected_count, 2):
        raise ValueError("posterior query points do not match query_shape")
    grid = points.reshape((*snapshot.query_shape, 2))
    grid_x = grid[..., 0]
    grid_y = grid[..., 1]
    expected_x, expected_y = np.meshgrid(grid_x[0, :], grid_y[:, 0])
    if not np.allclose(grid_x, expected_x) or not np.allclose(grid_y, expected_y):
        raise ValueError("posterior query points must form a structured grid")
    return grid_x, grid_y


def _ground_importance_grid(snapshot: PosteriorSnapshot) -> np.ndarray:
    """Return the exact common density supplied before ground Voronoi masks."""
    density = np.asarray(snapshot.density, dtype=float)
    expected_shape = (int(np.prod(snapshot.query_shape)),)
    if density.shape != expected_shape:
        raise ValueError("posterior density must match query_shape")
    if not np.all(np.isfinite(density)) or np.any(density < 0.0):
        raise ValueError("posterior density must be finite and nonnegative")
    return density.reshape(snapshot.query_shape)


def _truth_grid(
    ground_truth: np.ndarray, grid_x: np.ndarray, grid_y: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    try:
        truth = np.asarray(ground_truth, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("ground_truth must be a numeric two-dimensional field") from exc
    if truth.ndim != 2 or min(truth.shape) < 2:
        raise ValueError("ground_truth must be a two-dimensional field")
    if not np.all(np.isfinite(truth)):
        raise ValueError("ground_truth must contain only finite values")
    x_values = np.linspace(float(np.min(grid_x)), float(np.max(grid_x)), truth.shape[1])
    y_values = np.linspace(float(np.min(grid_y)), float(np.max(grid_y)), truth.shape[0])
    truth_x, truth_y = np.meshgrid(x_values, y_values)
    return truth_x, truth_y, truth


def _plot_combined_trajectories(
    axis,
    aerial_trajectories,
    ground_trajectories,
    *,
    ground_fov_degrees: float | None = None,
    ground_sensing_range: float | None = None,
    ground_final_states: np.ndarray | None = None,
) -> None:
    _plot_team(axis, aerial_trajectories, "Aerial", "#2563eb")
    _plot_team(
        axis,
        ground_trajectories,
        "Ground",
        "#00e5ff",
        linewidth=2.8,
    )
    if ground_fov_degrees is not None or ground_sensing_range is not None:
        if ground_fov_degrees is None or ground_sensing_range is None:
            raise ValueError(
                "ground_fov_degrees and ground_sensing_range must be set together"
            )
        _plot_ground_sensing_footprints(
            axis,
            ground_trajectories,
            ground_fov_degrees,
            ground_sensing_range,
            ground_final_states,
        )
    if aerial_trajectories or ground_trajectories:
        axis.legend(loc="best", fontsize=8)
    else:
        axis.text(
            0.5,
            0.5,
            "No robots",
            transform=axis.transAxes,
            ha="center",
            va="center",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )


def _plot_ground_sensing_footprints(
    axis,
    trajectories,
    fov_degrees: float,
    sensing_range: float,
    final_states: np.ndarray | None,
) -> None:
    """Overlay heading-centered sensing sectors at final ground states."""
    fov = float(fov_degrees)
    depth = float(sensing_range)
    if not np.isfinite(fov) or not 0.0 < fov <= 360.0:
        raise ValueError("ground_fov_degrees must be finite and in (0, 360]")
    if not np.isfinite(depth) or depth <= 0.0:
        raise ValueError("ground_sensing_range must be finite and positive")
    if not trajectories:
        return
    if final_states is None:
        state_rows = []
        for trajectory in trajectories:
            values = np.asarray(trajectory, dtype=float)
            if values.ndim != 2 or values.shape[1] < 3 or values.shape[0] == 0:
                raise ValueError(
                    "ground final states are required when trajectories omit heading"
                )
            state_rows.append(values[-1, :3])
        states = np.asarray(state_rows, dtype=float)
    else:
        states = np.asarray(final_states, dtype=float)
    if (
        states.ndim != 2
        or states.shape[0] != len(trajectories)
        or states.shape[1] < 3
    ):
        raise ValueError(
            "ground_final_states must have shape (num_ground_robots, state_dim>=3)"
        )

    half_fov = 0.5 * fov
    for index, final_state in enumerate(states):
        if not np.all(np.isfinite(final_state[:3])):
            raise ValueError("ground final states must be finite")
        heading_degrees = float(np.rad2deg(final_state[2]))
        footprint = Wedge(
            center=(float(final_state[0]), float(final_state[1])),
            r=depth,
            theta1=heading_degrees - half_fov,
            theta2=heading_degrees + half_fov,
            facecolor="#00e5ff",
            edgecolor="#ffffff",
            linewidth=1.6,
            linestyle="--",
            alpha=0.24,
            zorder=2,
            label="Ground sensing footprints" if index == 0 else None,
        )
        axis.add_patch(footprint)


def _plot_team(
    axis,
    trajectories,
    label_prefix: str,
    color: str,
    *,
    linewidth: float = 2.0,
) -> None:
    final_positions = []
    for index, trajectory in enumerate(trajectories):
        values = np.asarray(trajectory, dtype=float)
        if values.ndim != 2 or values.shape[1] < 2 or values.shape[0] == 0:
            raise ValueError("trajectories must have shape (N, state_dim>=2)")
        if not np.all(np.isfinite(values[:, :2])):
            raise ValueError("trajectory positions must be finite")
        axis.plot(
            values[:, 0],
            values[:, 1],
            color=color,
            linewidth=linewidth,
            alpha=1.0,
            label=f"{label_prefix} trajectories" if index == 0 else None,
        )
        final_positions.append(values[-1, :2])
    if final_positions:
        states = np.asarray(final_positions)
        axis.scatter(
            states[:, 0],
            states[:, 1],
            s=58,
            marker="X",
            color=color,
            edgecolor="white",
            linewidth=0.9,
            zorder=3,
            label=f"{label_prefix} final states",
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        "-c",
        default="configs/multifidelity_plot_smoke.yaml",
    )
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/final_multifidelity_state_milestone.png"),
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_args()
    configuration = load_coupled_configuration(arguments.config)
    aerial_params = configuration.aerial
    ground_params = configuration.ground
    number_of_steps = (
        aerial_params.num_steps
        if arguments.num_steps is None
        else arguments.num_steps
    )
    seed = aerial_params.random_seed if arguments.seed is None else arguments.seed
    simulation = build_coupled_simulation(
        aerial_params, ground_params, seed=seed
    )
    result = simulation.run(number_of_steps)
    snapshot = simulation.latest_posterior
    if snapshot is None:
        raise RuntimeError("simulation produced no valid final posterior")
    path = render_final_multifidelity_state(
        result,
        snapshot,
        simulation.hedac.goal_density,
        arguments.output,
        ground_fov_degrees=ground_params.fov_deg,
        ground_sensing_range=ground_params.fov_depth,
        ground_final_states=simulation.ground_team.get_states(),
    )
    print(f"saved={path}")
    print(f"posterior_version={snapshot.version}")
    print(
        "kernel_hyperparameters="
        f"({snapshot.low_kernel_length_scale:.12g}, "
        f"{snapshot.low_kernel_variance:.12g}, "
        f"{snapshot.discrepancy_kernel_length_scale:.12g}, "
        f"{snapshot.discrepancy_kernel_variance:.12g})"
    )
    print(f"fit_performed_this_update={snapshot.hyperparameter_fit_performed}")
    print(f"aerial_trajectories={len(result.aerial_trajectories)}")
    print(f"ground_trajectories={len(result.ground_trajectories)}")


if __name__ == "__main__":
    main()
