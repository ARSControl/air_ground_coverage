"""Compare previous softplus and current clipped-mean Lloyd densities."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import numpy as np
from scipy.interpolate import RegularGridInterpolator

from src.core.base import HEDACParams
from src.core.density import normalize_nonnegative_density
from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import _GroundLloydController, build_coupled_simulation
from src.models.agents import AgentTeam, UnicycleAgent


STEPS = 60
INITIAL_STATES = (
    (np.array([1.0, 1.0]), np.pi / 2.0),
    (np.array([4.5, 1.0]), np.pi / 2.0),
    (np.array([8.0, 1.0]), np.pi / 2.0),
)
COLORS = ("#ff9f1c", "#2ec4b6", "#e71d36")


def _lloyd_params() -> HEDACParams:
    return HEDACParams.from_dict(
        {
            "simulation": {"num_agents": 3, "dt": 0.1},
            "controller": {"type": "lloyd"},
            "agents": {
                "model_type": "unicycle",
                "max_velocity": 1.0,
                "max_angular_velocity": 1.5,
                "dt_agent": 0.1,
            },
            "sensor": {"fov_depth": 2.0, "fov_degrees": 360.0},
            "gpr": {"obs_per_step": 1},
            "map": {"local_grid_points": 9},
            "lloyd": {
                "position_gain": 1.0,
                "heading_gain": 2.0,
                "centroid_tolerance": 0.02,
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 1.5,
            },
        }
    )


def _team() -> AgentTeam:
    return AgentTeam(
        [
            UnicycleAgent(
                position,
                heading,
                max_v=1.0,
                max_omega=1.5,
                dt=0.1,
                agent_id=index,
            )
            for index, (position, heading) in enumerate(INITIAL_STATES)
        ]
    )


def _posterior():
    configuration = load_coupled_configuration(
        "configs/multifidelity_plot_smoke.yaml"
    )
    simulation = build_coupled_simulation(
        configuration.aerial, configuration.ground, seed=77
    )
    simulation.run(12)
    snapshot = simulation.latest_posterior
    if snapshot is None:
        raise RuntimeError("the deterministic coupled run published no posterior")
    return snapshot


def _run_lloyd(density: np.ndarray):
    team = _team()
    controller = _GroundLloydController(
        _lloyd_params(), np.zeros((9, 9)), team, np.zeros((9, 9))
    )
    for step_num in range(STEPS):
        controller.step(step_num, None, external_density=density)
    return team, controller


def _interest_history(team: AgentTeam, snapshot) -> np.ndarray:
    points = np.asarray(snapshot.query_points)
    x_axis = np.unique(points[:, 0])
    y_axis = np.unique(points[:, 1])
    interpolator = RegularGridInterpolator(
        (y_axis, x_axis),
        np.asarray(snapshot.high_mean).reshape(snapshot.query_shape),
        bounds_error=True,
    )
    histories = team.get_histories()
    values = []
    for step_num in range(histories[0].shape[0]):
        positions = np.asarray([history[step_num] for history in histories])
        values.append(np.mean(interpolator(positions[:, [1, 0]])))
    return np.asarray(values)


def _trajectory_panel(axis, density, team, title):
    image = axis.imshow(
        density.reshape((9, 9)),
        origin="lower",
        extent=(0.0, 9.0, 0.0, 9.0),
        cmap="viridis",
        interpolation="bilinear",
    )
    for index, (history, color) in enumerate(zip(team.get_histories(), COLORS)):
        axis.plot(
            history[:, 0],
            history[:, 1],
            color=color,
            linewidth=2.2,
            label=f"ground {index + 1}",
        )
        axis.scatter(
            history[0, 0],
            history[0, 1],
            facecolor="white",
            edgecolor=color,
            s=40,
            linewidth=1.5,
            zorder=4,
        )
        axis.scatter(
            history[-1, 0],
            history[-1, 1],
            marker="*",
            color=color,
            edgecolor="black",
            linewidth=0.4,
            s=110,
            zorder=5,
        )
    axis.set_title(title)
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.set_xlim(0.0, 9.0)
    axis.set_ylim(0.0, 9.0)
    axis.set_aspect("equal")
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/ground_mean_density_comparison.png"),
    )
    args = parser.parse_args()

    snapshot = _posterior()
    mean = np.asarray(snapshot.high_mean)
    previous_softplus_density = normalize_nonnegative_density(
        np.maximum(mean, 0.0) + np.log1p(np.exp(-np.abs(mean))),
        snapshot.integration_weights,
        tolerance=1.0e-9,
    )
    positive_mean_density = np.asarray(snapshot.density)
    expected_positive_mean = normalize_nonnegative_density(
        np.maximum(mean, 0.0),
        snapshot.integration_weights,
        tolerance=1.0e-9,
    )
    np.testing.assert_array_equal(positive_mean_density, expected_positive_mean)
    softplus_team, softplus_controller = _run_lloyd(previous_softplus_density)
    mean_team, mean_controller = _run_lloyd(positive_mean_density)
    softplus_interest = _interest_history(softplus_team, snapshot)
    mean_interest = _interest_history(mean_team, snapshot)
    peak_position = snapshot.query_points[int(np.argmax(mean))]

    figure, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)
    mean_map = mean.reshape(snapshot.query_shape)
    mean_limit = float(np.max(np.abs(mean_map)))
    raw_image = axes[0, 0].imshow(
        mean_map,
        origin="lower",
        extent=(0.0, 9.0, 0.0, 9.0),
        cmap="coolwarm",
        norm=TwoSlopeNorm(vmin=-mean_limit, vcenter=0.0, vmax=mean_limit),
        interpolation="bilinear",
    )
    axes[0, 0].scatter(
        peak_position[0], peak_position[1], marker="x", color="black", s=70
    )
    axes[0, 0].set_title(
        "A. Real GP posterior mean\n"
        f"range [{mean.min():.3f}, {mean.max():.3f}], "
        f"{np.count_nonzero(mean < 0)} negative cells"
    )
    axes[0, 0].set_xlabel("x")
    axes[0, 0].set_ylabel("y")
    axes[0, 0].set_aspect("equal")
    figure.colorbar(raw_image, ax=axes[0, 0], shrink=0.82, label="posterior mean")

    soft_image = _trajectory_panel(
        axes[0, 1],
        previous_softplus_density,
        softplus_team,
        "B. Previous normalized softplus\n"
        "density max/min = "
        f"{previous_softplus_density.max() / previous_softplus_density.min():.2f}",
    )
    figure.colorbar(
        soft_image, ax=axes[0, 1], shrink=0.82, label="softplus density"
    )
    mean_image = _trajectory_panel(
        axes[1, 0],
        positive_mean_density,
        mean_team,
        "C. Current normalized clipped GP mean\n"
        f"{np.count_nonzero(positive_mean_density == 0)} zero-density cells",
    )
    figure.colorbar(
        mean_image, ax=axes[1, 0], shrink=0.82, label="positive-mean density"
    )

    step_axis = np.arange(STEPS + 1)
    axes[1, 1].plot(
        step_axis,
        softplus_interest,
        color="#7b2cbf",
        linewidth=2.2,
        label="softplus-driven Lloyd",
    )
    axes[1, 1].plot(
        step_axis,
        mean_interest,
        color="#d62828",
        linewidth=2.2,
        label="positive-mean-driven Lloyd",
    )
    axes[1, 1].set_title("D. Mean GP interest at robot positions")
    axes[1, 1].set_xlabel("controller step")
    axes[1, 1].set_ylabel("posterior mean sampled at robots")
    axes[1, 1].grid(alpha=0.25)
    axes[1, 1].legend()
    figure.suptitle(
        "Ground Lloyd response: previous softplus vs current clipped GP mean",
        fontsize=14,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)

    softplus_positions = softplus_team.get_states()[:, :2]
    mean_positions = mean_team.get_states()[:, :2]
    print(f"saved={args.output}")
    print(f"posterior_version={snapshot.version}")
    print(f"posterior_mean_min={mean.min():.12f}")
    print(f"posterior_mean_max={mean.max():.12f}")
    print(f"posterior_negative_cells={np.count_nonzero(mean < 0)}")
    print(
        "previous_softplus_density_max_min_ratio="
        f"{previous_softplus_density.max() / previous_softplus_density.min():.12f}"
    )
    print(f"softplus_final_mean_interest={softplus_interest[-1]:.12f}")
    print(f"positive_mean_final_mean_interest={mean_interest[-1]:.12f}")
    print(
        "softplus_final_mean_distance_to_peak="
        f"{np.mean(np.linalg.norm(softplus_positions - peak_position, axis=1)):.12f}"
    )
    print(
        "positive_mean_final_mean_distance_to_peak="
        f"{np.mean(np.linalg.norm(mean_positions - peak_position, axis=1)):.12f}"
    )
    print(f"softplus_centroid_residual={np.mean(np.linalg.norm(softplus_controller.last_centroids - softplus_positions, axis=1)):.12f}")
    print(f"positive_mean_centroid_residual={np.mean(np.linalg.norm(mean_controller.last_centroids - mean_positions, axis=1)):.12f}")


if __name__ == "__main__":
    main()
