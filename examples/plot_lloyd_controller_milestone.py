"""Compare the selectable MPC and Lloyd ground controllers deterministically."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.core.base import HEDACParams
from src.coupled_simulation import (
    _GroundLloydController,
    _LegacyGroundMPC,
    compute_weighted_voronoi_centroids,
)
from src.models.agents import AgentTeam, UnicycleAgent
from src.utils.voronoi import compute_voronoi_partitioning


MAP_SHAPE = (12, 12)
INITIAL_STATES = (
    (np.array([2.0, 2.0]), 0.2),
    (np.array([2.5, 9.5]), -0.4),
)
STEPS = 35


def _params(controller_type: str) -> HEDACParams:
    return HEDACParams.from_dict(
        {
            "controller": {"type": controller_type},
            "simulation": {"num_agents": 2, "dt": 0.1},
            "agents": {
                "model_type": "unicycle",
                "max_velocity": 1.5,
                "max_acceleration": 1.5,
                "max_angular_velocity": 1.5,
                "max_angular_acceleration": 1.5,
                "dt_agent": 0.1,
            },
            "sensor": {"fov_degrees": 360.0, "fov_depth": 4.0},
            "multi_agent": {"sensing_range": 4.0},
            "gpr": {"obs_per_step": 1},
            "map": {"resolution": 1.0, "local_grid_points": 21},
            "mpc": {
                "horizon": 2,
                "tolerance": 1.0e-5,
                "max_iterations": 100,
                "print_time": False,
            },
            "lloyd": {
                "position_gain": 0.8,
                "heading_gain": 2.0,
                "centroid_tolerance": 0.03,
                "max_linear_velocity": 1.5,
                "max_angular_velocity": 1.5,
            },
        }
    )


def _team() -> AgentTeam:
    return AgentTeam(
        [
            UnicycleAgent(
                x0=position,
                theta0=heading,
                max_v=1.5,
                max_omega=1.5,
                dt=0.1,
                agent_id=index,
                observations_range=4,
                observations_count=1,
                fov_degrees=360.0,
            )
            for index, (position, heading) in enumerate(INITIAL_STATES)
        ]
    )


def _density(points: np.ndarray) -> np.ndarray:
    first = np.exp(
        -0.5
        * (
            ((points[:, 0] - 9.0) / 1.4) ** 2
            + ((points[:, 1] - 3.0) / 1.2) ** 2
        )
    )
    second = 0.8 * np.exp(
        -0.5
        * (
            ((points[:, 0] - 8.5) / 1.2) ** 2
            + ((points[:, 1] - 9.0) / 1.5) ** 2
        )
    )
    baseline = 0.02
    values = first + second + baseline
    return values / np.max(values)


def _centroid_residual(controller, density: np.ndarray) -> float:
    positions = controller.team.get_states()[:, :2]
    masks = compute_voronoi_partitioning(
        controller.query_points,
        positions,
        math.hypot(*MAP_SHAPE) + 1.0,
    )
    centroids, _ = compute_weighted_voronoi_centroids(
        controller.query_points,
        density,
        masks,
        controller.integration_weights,
        positions,
    )
    return float(np.mean(np.linalg.norm(centroids - positions, axis=1)))


def _run(controller, density: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
    residuals = [_centroid_residual(controller, density)]
    for step_num in range(STEPS):
        controller.step(step_num, None, external_density=density)
        residuals.append(_centroid_residual(controller, density))
    trajectories = [history.copy() for history in controller.team.get_histories()]
    return trajectories, np.asarray(residuals)


def _trajectory_panel(
    axis,
    density_map: np.ndarray,
    trajectories: list[np.ndarray],
    controller,
    title: str,
) -> None:
    image = axis.imshow(
        density_map,
        origin="lower",
        extent=(0.0, MAP_SHAPE[1], 0.0, MAP_SHAPE[0]),
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        interpolation="bilinear",
    )
    colors = ("#ff9f1c", "#2ec4b6")
    for index, (trajectory, color) in enumerate(zip(trajectories, colors)):
        axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            color=color,
            linewidth=2.4,
            label=f"robot {index + 1}",
        )
        axis.scatter(
            trajectory[0, 0],
            trajectory[0, 1],
            marker="o",
            s=45,
            facecolor="white",
            edgecolor=color,
            linewidth=1.5,
            zorder=4,
        )
        axis.scatter(
            trajectory[-1, 0],
            trajectory[-1, 1],
            marker="*",
            s=120,
            color=color,
            edgecolor="black",
            linewidth=0.5,
            zorder=5,
        )
    positions = controller.team.get_states()[:, :2]
    masks = compute_voronoi_partitioning(
        controller.query_points,
        positions,
        math.hypot(*MAP_SHAPE) + 1.0,
    )
    centroids, _ = compute_weighted_voronoi_centroids(
        controller.query_points,
        controller.last_density,
        masks,
        controller.integration_weights,
        positions,
    )
    for position, centroid, color in zip(positions, centroids, colors):
        axis.plot(
            [position[0], centroid[0]],
            [position[1], centroid[1]],
            color=color,
            linestyle=":",
            linewidth=1.4,
        )
        axis.scatter(
            centroid[0],
            centroid[1],
            marker="x",
            s=65,
            color=color,
            linewidth=2.0,
            zorder=5,
        )
    axis.set_title(title)
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.set_xlim(0.0, MAP_SHAPE[1])
    axis.set_ylim(0.0, MAP_SHAPE[0])
    axis.set_aspect("equal")
    axis.legend(loc="upper left", framealpha=0.9)
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/lloyd_controller_milestone.png"),
    )
    args = parser.parse_args()

    map_array = np.zeros(MAP_SHAPE, dtype=int)
    high_field = np.ones(MAP_SHAPE)
    mpc = _LegacyGroundMPC(_params("mpc"), map_array, _team(), high_field)
    lloyd = _GroundLloydController(
        _params("lloyd"), map_array, _team(), high_field
    )
    density = _density(mpc.query_points)
    mpc_trajectories, mpc_residual = _run(mpc, density)
    lloyd_trajectories, lloyd_residual = _run(lloyd, density)

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    density_map = density.reshape(mpc.query_shape)
    image = _trajectory_panel(
        axes[0],
        density_map,
        mpc_trajectories,
        mpc,
        "A. Existing limited-FOV MPC",
    )
    _trajectory_panel(
        axes[1],
        density_map,
        lloyd_trajectories,
        lloyd,
        "B. New weighted Lloyd controller",
    )
    figure.colorbar(image, ax=axes[:2], shrink=0.82, label="common density")

    step_axis = np.arange(STEPS + 1)
    axes[2].plot(step_axis, mpc_residual, label="MPC", color="#7b2cbf", linewidth=2)
    axes[2].plot(
        step_axis, lloyd_residual, label="Lloyd", color="#d62828", linewidth=2
    )
    axes[2].set_title("C. Mean distance to current cell centroid")
    axes[2].set_xlabel("controller step")
    axes[2].set_ylabel("centroid residual")
    axes[2].grid(alpha=0.25)
    axes[2].legend()

    figure.suptitle(
        "Selectable ground coverage laws on identical density and initial states",
        fontsize=14,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)

    print(f"saved={args.output}")
    print(f"mpc_residual_initial={mpc_residual[0]:.12f}")
    print(f"mpc_residual_final={mpc_residual[-1]:.12f}")
    print(f"lloyd_residual_initial={lloyd_residual[0]:.12f}")
    print(f"lloyd_residual_final={lloyd_residual[-1]:.12f}")
    print(f"mpc_legacy_gp_samples={mpc.gp.n_samples}")
    print(f"lloyd_legacy_gp_samples={lloyd.gp.n_samples}")


if __name__ == "__main__":
    main()
