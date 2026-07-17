"""Visualize deterministic shadow-mode simulation scheduling and estimation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.simulation import (  # noqa: E402
    CoordinatorEventType,
    build_multifidelity_coordinator,
)


@dataclass
class DemoAgent:
    id: int
    position: np.ndarray
    heading: float
    history: list[np.ndarray] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.history.append(self.position.copy())

    @property
    def theta(self) -> float:
        return self.heading

    @property
    def x_hist(self) -> np.ndarray:
        return np.asarray(self.history)


class DemoTeam:
    def __init__(self, agents: list[DemoAgent]) -> None:
        self.agents = agents

    def get_histories(self) -> list[np.ndarray]:
        return [agent.x_hist for agent in self.agents]


class DemoAerialController:
    """Deterministic motion stand-in; scheduling uses the production loop."""

    def __init__(self) -> None:
        self.step_count = 0

    def step(self, team: DemoTeam, step_num: int) -> float:
        self.step_count += 1
        for index, agent in enumerate(team.agents):
            direction = np.array([0.16, 0.05 * (-1) ** index])
            agent.position = np.clip(agent.position + direction, 0.0, 20.0)
            agent.heading = float(np.arctan2(direction[1], direction[0]))
            agent.history.append(agent.position.copy())
        return float(1.0 / (step_num + 1.0))


class DemoGroundController:
    """Deterministic legacy-path stand-in used only for the visual summary."""

    def __init__(self, team: DemoTeam) -> None:
        self.team = team
        self.step_count = 0

    @property
    def density_source(self) -> str:
        return "legacy_posthoc_fusion"

    def step(self, step_num: int, hedac: DemoAerialController) -> None:
        del step_num, hedac
        self.step_count += 1
        for index, agent in enumerate(self.team.agents):
            direction = np.array([-0.04 * (-1) ** index, 0.10])
            agent.position = np.clip(agent.position + direction, 0.0, 20.0)
            agent.heading = float(np.arctan2(direction[1], direction[0]))
            agent.history.append(agent.position.copy())


class HistoricalShadowSimulation:
    """Frozen Milestone 5 phase sequence used only to reproduce its artifact."""

    def __init__(
        self,
        aerial_team: DemoTeam,
        hedac: DemoAerialController,
        ground_team: DemoTeam,
        ground_controller: DemoGroundController,
        coordinator,
        dt: float,
    ) -> None:
        self.aerial_team = aerial_team
        self.hedac = hedac
        self.ground_team = ground_team
        self.ground_controller = ground_controller
        self.coordinator = coordinator
        self.dt = dt

    @property
    def latest_posterior(self):
        return self.coordinator.latest_posterior

    def run(self, num_steps: int):
        reports = []
        step_results = []
        for step_num in range(num_steps):
            simulation_time = step_num * self.dt
            reports.append(
                self.coordinator.collect_low_if_due(
                    simulation_time, self.aerial_team
                )
            )
            self.hedac.step(self.aerial_team, step_num)
            reports.append(
                self.coordinator.collect_high_if_due(
                    simulation_time, self.ground_team
                )
            )
            reports.append(self.coordinator.update_if_due(simulation_time))
            self.ground_controller.step(step_num, self.hedac)
            step_results.append(
                SimpleNamespace(
                    simulation_time=simulation_time,
                    posterior_version=self.coordinator.estimator.version,
                )
            )
        return SimpleNamespace(
            step_results=tuple(step_results),
            estimator_reports=tuple(reports),
            aerial_trajectories=tuple(
                np.asarray(history) for history in self.aerial_team.get_histories()
            ),
            ground_trajectories=tuple(
                np.asarray(history) for history in self.ground_team.get_histories()
            ),
        )


def build_demo() -> tuple[
    HistoricalShadowSimulation, np.ndarray, np.ndarray, np.ndarray
]:
    coordinates = np.linspace(0.0, 20.0, 21)
    grid_x, grid_y = np.meshgrid(coordinates, coordinates)
    high_truth = (
        np.exp(-((grid_x - 7.0) ** 2 + (grid_y - 12.0) ** 2) / 22.0)
        + 0.8 * np.exp(-((grid_x - 15.0) ** 2 + (grid_y - 7.0) ** 2) / 5.0)
    )
    query_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    aerial_config = {
        "simulation": {"random_seed": 505},
        "sensor": {"fov_depth": 2.5, "fov_degrees": 360.0},
        "gpr": {"obs_per_step": 1},
        "multifidelity": {
            "rho": 0.8,
            "aerial_sensor_period": 0.4,
            "ground_sensor_period": 0.2,
            "gp_update_period": 0.5,
            "low_kernel": {"length_scale": 4.0, "variance": 1.0},
            "discrepancy_kernel": {"length_scale": 1.4, "variance": 0.4},
            "low_noise_variance": 0.01,
            "high_noise_variance": 0.001,
            "jitter": 1.0e-9,
            "retention": {
                "max_low_samples": 40,
                "max_high_samples": 40,
                "min_low_separation": 0.0,
                "min_high_separation": 0.0,
            },
            "density": {"normalization_tolerance": 1.0e-9},
            "sensor": {
                "low_fidelity_smoothing_sigma_cells": 1.5,
                "random_seed_offset": 200,
            },
        },
    }
    ground_config = {
        "sensor": {"fov_depth": 2.0, "fov_degrees": 100.0},
        "gpr": {"obs_per_step": 1},
    }
    coordinator = build_multifidelity_coordinator(
        aerial_config,
        ground_config,
        high_truth,
        query_points,
        grid_x.shape,
        np.ones(query_points.shape[0]),
    )
    aerial_team = DemoTeam(
        [
            DemoAgent(0, np.array([3.0, 5.0]), 0.0),
            DemoAgent(1, np.array([4.0, 15.0]), 0.0),
        ]
    )
    ground_team = DemoTeam(
        [
            DemoAgent(10, np.array([16.0, 3.0]), np.pi / 2.0),
            DemoAgent(11, np.array([13.0, 9.0]), np.pi / 2.0),
        ]
    )
    simulation = HistoricalShadowSimulation(
        aerial_team=aerial_team,
        hedac=DemoAerialController(),
        ground_team=ground_team,
        ground_controller=DemoGroundController(ground_team),
        coordinator=coordinator,
        dt=0.1,
    )
    return simulation, grid_x, grid_y, high_truth


def generate_plot(output_path: Path) -> Path:
    simulation, grid_x, grid_y, high_truth = build_demo()
    result = simulation.run(21)
    snapshot = simulation.latest_posterior
    assert snapshot is not None
    times = np.asarray([step.simulation_time for step in result.step_results])
    versions = np.asarray([step.posterior_version for step in result.step_results])

    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    figure.suptitle(
        "Simulation integration in shadow mode: controllers stay on legacy densities",
        fontsize=15,
        fontweight="bold",
    )
    colors = {
        CoordinatorEventType.LOW_COLLECTION: "#2563eb",
        CoordinatorEventType.HIGH_COLLECTION: "#dc2626",
        CoordinatorEventType.ESTIMATOR_UPDATE: "#059669",
    }

    event_axis = axes[0, 0]
    rows = {
        CoordinatorEventType.LOW_COLLECTION: 2,
        CoordinatorEventType.HIGH_COLLECTION: 1,
        CoordinatorEventType.ESTIMATOR_UPDATE: 0,
    }
    for event_type, row in rows.items():
        event_times = [
            report.simulation_time
            for report in result.estimator_reports
            if report.event_type is event_type and report.fired
        ]
        event_axis.scatter(
            event_times,
            np.full(len(event_times), row),
            color=colors[event_type],
            s=55,
            label=event_type.value.replace("_", " ").upper(),
        )
    event_axis.set_yticks([0, 1, 2], ["GP UPDATE", "HIGH", "LOW"])
    event_axis.set_title("A. Three independent simulated-time schedules")
    event_axis.set_xlabel("Simulation time")
    event_axis.set_ylim(-0.5, 2.5)
    event_axis.legend(loc="upper right", fontsize=8)

    progress_axis = axes[0, 1]
    controller_steps = np.arange(1, times.size + 1)
    progress_axis.plot(
        times,
        controller_steps,
        color="#111827",
        linewidth=2.5,
        label="Aerial and ground controller steps",
    )
    version_axis = progress_axis.twinx()
    version_axis.step(
        times,
        versions,
        where="post",
        color=colors[CoordinatorEventType.ESTIMATOR_UPDATE],
        linewidth=2.5,
        label="Cached posterior version",
    )
    progress_axis.set_title("B. Controllers continue between GP updates")
    progress_axis.set_xlabel("Simulation time")
    progress_axis.set_ylabel("Cumulative controller steps")
    version_axis.set_ylabel("Posterior version")
    handles, labels = progress_axis.get_legend_handles_labels()
    other_handles, other_labels = version_axis.get_legend_handles_labels()
    progress_axis.legend(handles + other_handles, labels + other_labels, fontsize=8)

    truth_axis = axes[1, 0]
    truth_image = truth_axis.contourf(grid_x, grid_y, high_truth, levels=20, cmap="magma")
    for index, trajectory in enumerate(result.aerial_trajectories):
        truth_axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            color="#60a5fa",
            linewidth=2,
            label="Aerial trajectories" if index == 0 else None,
        )
    for index, trajectory in enumerate(result.ground_trajectories):
        truth_axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            color="#f87171",
            linewidth=2,
            label="Ground trajectories" if index == 0 else None,
        )
    truth_axis.set_title("C. Simulator HIGH truth and deterministic motion")
    truth_axis.set_xlabel("x")
    truth_axis.set_ylabel("y")
    truth_axis.legend(loc="upper right", fontsize=8)
    figure.colorbar(truth_image, ax=truth_axis, shrink=0.8)

    posterior_axis = axes[1, 1]
    posterior_image = posterior_axis.contourf(
        grid_x,
        grid_y,
        snapshot.high_mean.reshape(snapshot.query_shape),
        levels=20,
        cmap="magma",
    )
    posterior_axis.set_title(
        f"D. Shadow posterior v{snapshot.version} (not consumed by controllers)"
    )
    posterior_axis.set_xlabel("x")
    posterior_axis.set_ylabel("y")
    figure.colorbar(posterior_image, ax=posterior_axis, shrink=0.8)

    for axis in axes.flat:
        axis.grid(True, alpha=0.2)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/shadow_simulation_milestone.png"),
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    print(generate_plot(arguments.output))
