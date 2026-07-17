"""Plot deterministic posterior-density feedback into the existing ground MPC."""

from __future__ import annotations

import argparse
from pathlib import Path

import casadi as ca
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.core import costFunctions
from src.core.base import HEDACParams
from src.core.observations import Fidelity, Observation
from src.coupled_simulation import _LegacyGroundMPC, _structured_query_grid
from src.models.agents import AgentTeam, UnicycleAgent
from src.simulation import build_multifidelity_coordinator


def _aerial_config() -> dict:
    return {
        "simulation": {"random_seed": 31},
        "sensor": {"fov_depth": 2.0, "fov_degrees": 360.0},
        "gpr": {"obs_per_step": 1},
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


def _ground_params() -> HEDACParams:
    return HEDACParams.from_dict(
        {
            "simulation": {"num_agents": 1, "dt": 0.1},
            "agents": {
                "model_type": "unicycle",
                "max_velocity": 1.0,
                "max_acceleration": 1.0,
                "max_angular_velocity": 1.0,
                "max_angular_acceleration": 1.0,
                "dt_agent": 0.1,
                "agent_radius": 1.0,
                "min_kernel_val": 0.01,
            },
            "sensor": {"fov_degrees": 120.0, "fov_depth": 3.0},
            "multi_agent": {"sensing_range": 3.0},
            "gpr": {"obs_per_step": 1, "min_samples": 10},
            "map": {"resolution": 1.0, "local_grid_points": 17},
            "mpc": {
                "horizon": 4,
                "tolerance": 1.0e-5,
                "max_iterations": 80,
                "print_time": False,
            },
        }
    )


def _posterior_densities():
    points, shape, integration_weights = _structured_query_grid((9, 9), 17)
    map_x = points[:, 0].reshape(shape)
    map_y = points[:, 1].reshape(shape)
    truth_x, truth_y = np.meshgrid(np.arange(9), np.arange(9))
    high_field = np.exp(-((truth_x - 6.5) ** 2 + (truth_y - 4.0) ** 2) / 0.8)
    coordinator = build_multifidelity_coordinator(
        _aerial_config(),
        _ground_params().to_dict(),
        high_field,
        points,
        shape,
        integration_weights,
        seed=31,
    )
    observations = []
    for index, (x, y) in enumerate(
        (x, y) for y in (1.0, 3.0, 5.0, 7.0) for x in (1.0, 3.0, 5.0, 7.0)
    ):
        value = float(np.exp(-((x - 2.0) ** 2 + (y - 4.0) ** 2) / 10.0))
        observations.append(
            Observation(0.0, f"low-{index}", (x, y), value, Fidelity.LOW, 0.01)
        )
    coordinator.estimator.submit_many(observations)
    coordinator.estimator.update(0.0)
    before = coordinator.ground_density(points, integration_weights)
    coordinator.estimator.submit(
        Observation(0.1, "ground-0", (6.5, 4.0), 4.0, Fidelity.HIGH, 0.0001)
    )
    coordinator.estimator.update(0.1)
    after = coordinator.ground_density(points, integration_weights)
    if before is None or after is None:
        raise RuntimeError("ground densities were not published")
    return points, shape, integration_weights, map_x, map_y, before, after


def _run_real_mpc(density: np.ndarray) -> tuple[np.ndarray, int]:
    params = _ground_params()
    agent = UnicycleAgent(
        x0=np.array([5.0, 4.0]),
        theta0=0.0,
        max_v=1.0,
        max_omega=1.0,
        dt=0.1,
        agent_id=0,
        observations_range=3,
        observations_count=1,
        fov_degrees=120.0,
    )
    team = AgentTeam([agent])
    controller = _LegacyGroundMPC(
        params, np.zeros((9, 9), dtype=int), team, np.ones((9, 9))
    )
    for step_num in range(15):
        controller.step(step_num, None, external_density=density)
    return team.get_histories()[0], controller.gp.n_samples


def _coverage_cost(
    points: np.ndarray, position: np.ndarray, density: np.ndarray
) -> float:
    symbolic_position = ca.SX.sym("plot_position", 2)
    symbolic_weights = ca.SX.sym("plot_weights", points.shape[0])
    function = ca.Function(
        "plot_ground_coverage_cost",
        [symbolic_position, symbolic_weights],
        [
            costFunctions.coverage_cost(
                symbolic_position, ca.DM(points), symbolic_weights
            )
        ],
    )
    return float(function(position, density))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/ground_feedback_milestone.png"),
    )
    args = parser.parse_args()

    points, shape, weights, map_x, map_y, before, after = _posterior_densities()
    trajectory_before, legacy_samples_before = _run_real_mpc(before)
    trajectory_after, legacy_samples_after = _run_real_mpc(after)
    density_before = before.reshape(shape)
    density_after = after.reshape(shape)
    difference = density_after - density_before
    density_limits = (
        min(density_before.min(), density_after.min()),
        max(density_before.max(), density_after.max()),
    )
    difference_limit = float(np.max(np.abs(difference)))

    figure, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    for panel, density, title in (
        (axes[0, 0], density_before, "A. Ground density v1: LOW data only"),
        (axes[0, 1], density_after, "B. Ground density v2: after ground HIGH"),
    ):
        image = panel.pcolormesh(
            map_x,
            map_y,
            density,
            shading="nearest",
            cmap="viridis",
            vmin=density_limits[0],
            vmax=density_limits[1],
        )
        panel.set_title(title)
        figure.colorbar(image, ax=panel, label="normalized high density")
    axes[0, 1].scatter(6.5, 4.0, marker="*", s=180, c="white", edgecolor="black")

    image = axes[1, 0].pcolormesh(
        map_x,
        map_y,
        difference,
        shading="nearest",
        cmap="coolwarm",
        vmin=-difference_limit,
        vmax=difference_limit,
    )
    axes[1, 0].scatter(6.5, 4.0, marker="*", s=180, c="gold", edgecolor="black")
    axes[1, 0].set_title("C. Ground MPC weight change before Voronoi masking")
    figure.colorbar(image, ax=axes[1, 0], label="density / weight change")

    axes[1, 1].pcolormesh(
        map_x, map_y, density_after, shading="nearest", cmap="viridis"
    )
    axes[1, 1].plot(
        trajectory_before[:, 0],
        trajectory_before[:, 1],
        "--",
        color="#2563eb",
        lw=2.5,
        label="MPC trajectory using density v1",
    )
    axes[1, 1].plot(
        trajectory_after[:, 0],
        trajectory_after[:, 1],
        "-",
        color="#f97316",
        lw=2.5,
        label="MPC trajectory using density v2",
    )
    axes[1, 1].scatter(5.0, 4.0, c="white", edgecolor="black", s=55, zorder=3)
    axes[1, 1].scatter(6.5, 4.0, marker="*", s=180, c="white", edgecolor="black")
    axes[1, 1].set_title("D. Existing ground MPC response (15 steps)")
    axes[1, 1].legend(loc="best")

    for panel in axes.ravel():
        panel.set_xlabel("x")
        panel.set_ylabel("y")
        panel.set_xlim(0, 9)
        panel.set_ylim(0, 9)

    density_change = float(np.linalg.norm(after - before))
    trajectory_change = float(np.linalg.norm(trajectory_after - trajectory_before))
    before_cost = _coverage_cost(points, np.array([6.5, 4.0]), before)
    after_cost = _coverage_cost(points, np.array([6.5, 4.0]), after)
    before_mass = float(np.sum(before * weights))
    after_mass = float(np.sum(after * weights))
    figure.suptitle(
        "Ground HIGH → normalized posterior density → existing Voronoi/MPC path\n"
        f"L2 changes: density={density_change:.5f}, trajectory={trajectory_change:.5f}; "
        f"coverage cost at HIGH location {before_cost:.2f} → {after_cost:.2f}",
        fontsize=14,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, facecolor="white")
    plt.close(figure)
    print(f"saved={args.output}")
    print(f"density_l2_change={density_change:.12f}")
    print(f"trajectory_l2_change={trajectory_change:.12f}")
    print(f"coverage_cost_before={before_cost:.12f}")
    print(f"coverage_cost_after={after_cost:.12f}")
    print(f"weighted_mass_before={before_mass:.12f}")
    print(f"weighted_mass_after={after_mass:.12f}")
    print(
        "legacy_gp_samples="
        f"{legacy_samples_before},{legacy_samples_after} (expected 0,0)"
    )


if __name__ == "__main__":
    main()
