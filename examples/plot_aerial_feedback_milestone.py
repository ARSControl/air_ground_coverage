"""Plot deterministic ground-observation-to-HEDAC feedback for Milestone 6."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.core.observations import Fidelity, Observation
from src.models.agents import AgentTeam, DubinsAgent
from src.simulation import build_multifidelity_coordinator


def _config() -> dict:
    return {
        "simulation": {"random_seed": 19},
        "heat_equation": {
            "alpha": 0.1,
            "source_strength": 1.0,
            "beta": 0.01,
            "local_cooling": 0.1,
        },
        "agents": {
            "max_velocity": 1.0,
            "max_acceleration": 0.5,
            "max_angular_velocity": 0.785,
            "max_angular_acceleration": 0.393,
            "dt_agent": 0.1,
            "agent_radius": 1.0,
            "min_kernel_val": 0.01,
        },
        "sensor": {"fov_depth": 2.0, "fov_degrees": 360.0},
        "gpr": {"obs_per_step": 1, "min_samples": 100},
        "map": {"size": [9, 9], "resolution": 1.0},
        "visualization": {"gp_debug": False},
        "multifidelity": {
            "rho": 0.8,
            "aerial_sensor_period": 1.0,
            "ground_sensor_period": 1.0,
            "gp_update_period": 1.0,
            "low_kernel": {"length_scale": 2.4, "variance": 1.0},
            "discrepancy_kernel": {"length_scale": 0.8, "variance": 2.0},
            "low_noise_variance": 0.01,
            "high_noise_variance": 0.0001,
            "jitter": 1.0e-9,
            "retention": {
                "max_low_samples": 100,
                "max_high_samples": 100,
                "min_low_separation": 0.0,
                "min_high_separation": 0.0,
            },
            "aerial_target": {
                "lambda_interest": 1.0,
                "lambda_uncertainty": 0.25,
            },
            "density": {"normalization_tolerance": 1.0e-9},
            "sensor": {"low_fidelity_smoothing_sigma_cells": 1.0},
        },
    }


def _posterior_and_targets():
    query_axis = np.linspace(0.0, 8.0, 17)
    grid_x, grid_y = np.meshgrid(query_axis, query_axis)
    query_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    high_field = np.exp(-((grid_x - 6.5) ** 2 + (grid_y - 4.0) ** 2) / 0.8)
    ground_config = {
        "sensor": {"fov_depth": 2.0, "fov_degrees": 90.0},
        "gpr": {"obs_per_step": 1},
    }
    coordinator = build_multifidelity_coordinator(
        _config(),
        ground_config,
        high_field,
        query_points,
        grid_x.shape,
        np.ones(query_points.shape[0]),
        seed=19,
    )
    low_observations = []
    for index, (x, y) in enumerate(
        (x, y) for y in (1.0, 3.0, 5.0, 7.0) for x in (1.0, 3.0, 5.0, 7.0)
    ):
        value = float(np.exp(-((x - 2.0) ** 2 + (y - 4.0) ** 2) / 10.0))
        low_observations.append(
            Observation(0.0, f"low-{index}", (x, y), value, Fidelity.LOW, 0.01)
        )
    coordinator.estimator.submit_many(low_observations)
    coordinator.estimator.update(0.0)
    before = coordinator.latest_posterior
    if before is None:
        raise RuntimeError("LOW-only posterior was not published")
    axis = np.arange(9, dtype=float)
    weights = np.ones((9, 9), dtype=float)
    mask = np.ones((9, 9), dtype=bool)
    target_before = coordinator.aerial_target(axis, axis, weights, mask)

    coordinator.estimator.submit(
        Observation(0.1, "ground-0", (6.5, 4.0), 4.0, Fidelity.HIGH, 0.0001)
    )
    coordinator.estimator.update(0.1)
    after = coordinator.latest_posterior
    target_after = coordinator.aerial_target(axis, axis, weights, mask)
    if after is None or target_before is None or target_after is None:
        raise RuntimeError("feedback posterior or target was not published")
    return query_axis, before, after, target_before, target_after


def _trajectory(goal: np.ndarray, external_target: np.ndarray) -> np.ndarray:
    params = HEDACParams.from_dict(_config())
    map_loader = MapLoader(size=(9, 9), resolution=1.0)
    agent = DubinsAgent(
        x0=np.array([5.5, 3.5]),
        theta0=np.pi / 2.0,
        forward_speed=0.75,
        max_bank_angle=45.0,
        dt=0.1,
        agent_id=0,
        observations_range=2,
        observations_count=1,
    )
    team = AgentTeam([agent])
    hedac = HEDACAlgorithm(params, map_loader, goal)
    for step_num in range(30):
        hedac.step(
            team,
            step_num,
            external_goal_density=external_target,
            update_legacy_gp=False,
        )
    return team.get_histories()[0]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/aerial_feedback_milestone.png"),
    )
    args = parser.parse_args()

    axis, before, after, target_before, target_after = _posterior_and_targets()
    trajectory_before = _trajectory(target_before, target_before)
    trajectory_after = _trajectory(target_before, target_after)
    mean_before = before.high_mean.reshape(before.query_shape)
    mean_after = after.high_mean.reshape(after.query_shape)
    target_difference = target_after - target_before
    mean_limits = (min(mean_before.min(), mean_after.min()), mean_after.max())
    difference_limit = float(np.max(np.abs(target_difference)))

    figure, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    image = axes[0, 0].imshow(
        mean_before,
        origin="lower",
        extent=[axis[0], axis[-1], axis[0], axis[-1]],
        vmin=mean_limits[0],
        vmax=mean_limits[1],
        cmap="viridis",
    )
    axes[0, 0].set_title("A. Joint high latent mean: LOW data only (v1)")
    figure.colorbar(image, ax=axes[0, 0], label="latent mean")

    image = axes[0, 1].imshow(
        mean_after,
        origin="lower",
        extent=[axis[0], axis[-1], axis[0], axis[-1]],
        vmin=mean_limits[0],
        vmax=mean_limits[1],
        cmap="viridis",
    )
    axes[0, 1].scatter(6.5, 4.0, marker="*", s=180, c="white", edgecolor="black")
    axes[0, 1].set_title("B. Joint high latent mean: after ground HIGH (v2)")
    figure.colorbar(image, ax=axes[0, 1], label="latent mean")

    image = axes[1, 0].imshow(
        target_difference,
        origin="lower",
        extent=[0, 8, 0, 8],
        vmin=-difference_limit,
        vmax=difference_limit,
        cmap="coolwarm",
    )
    axes[1, 0].scatter(6.5, 4.0, marker="*", s=180, c="gold", edgecolor="black")
    axes[1, 0].set_title("C. Diagnostic only: cached target v2 - v1")
    figure.colorbar(image, ax=axes[1, 0], label="probability-mass change")

    axes[1, 1].imshow(
        target_after, origin="lower", extent=[0, 8, 0, 8], cmap="magma"
    )
    axes[1, 1].plot(
        trajectory_before[:, 0],
        trajectory_before[:, 1],
        "--",
        color="#2563eb",
        lw=2.5,
        label="trajectory generated with target v1",
    )
    axes[1, 1].plot(
        trajectory_after[:, 0],
        trajectory_after[:, 1],
        "-",
        color="#f97316",
        lw=2.5,
        label="trajectory generated with target v2",
    )
    axes[1, 1].scatter(6.5, 4.0, marker="*", s=180, c="white", edgecolor="black")
    axes[1, 1].scatter(5.5, 3.5, c="white", edgecolor="black", s=55, zorder=3)
    axes[1, 1].set_title("D. Both trajectories; background is absolute target v2")
    axes[1, 1].legend(loc="best")

    for panel in axes.ravel():
        panel.set_xlabel("x")
        panel.set_ylabel("y")
        panel.set_xlim(0, 8)
        panel.set_ylim(0, 8)

    mean_change = float(np.linalg.norm(after.high_mean - before.high_mean))
    variance_change = float(
        np.linalg.norm(after.high_variance - before.high_variance)
    )
    target_change = float(np.linalg.norm(target_difference))
    trajectory_change = float(np.linalg.norm(trajectory_after - trajectory_before))
    figure.suptitle(
        "Ground HIGH → posterior → aerial target → HEDAC trajectory\n"
        f"L2 changes: mean={mean_change:.3f}, variance={variance_change:.3f}, "
        f"target={target_change:.5f}, trajectory={trajectory_change:.5f}",
        fontsize=14,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, facecolor="white")
    plt.close(figure)
    print(f"saved={args.output}")
    print(f"posterior_mean_l2_change={mean_change:.12f}")
    print(f"posterior_variance_l2_change={variance_change:.12f}")
    print(f"aerial_target_l2_change={target_change:.12f}")
    print(f"trajectory_l2_change={trajectory_change:.12f}")


if __name__ == "__main__":
    main()
