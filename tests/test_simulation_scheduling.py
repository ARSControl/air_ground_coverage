"""Integration tests for deterministic coordinator and coupled-loop scheduling."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from src.core.observations import Fidelity, Observation
from src.coupled_simulation import CoupledSimulation
from src.simulation import (
    CoordinatorEventType,
    EstimatorMode,
    PeriodicEvent,
    build_multifidelity_coordinator,
)


@dataclass
class FakeAgent:
    id: int
    position: np.ndarray
    heading: float = 0.0
    history: list[np.ndarray] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.history.append(self.position.copy())

    @property
    def theta(self) -> float:
        return self.heading

    @property
    def x_hist(self) -> np.ndarray:
        return np.asarray(self.history)


class FakeTeam:
    def __init__(self, agents: list[FakeAgent]) -> None:
        self.agents = agents

    def get_histories(self) -> list[np.ndarray]:
        return [agent.x_hist for agent in self.agents]


class FakeHedac:
    def __init__(self) -> None:
        self.step_count = 0
        self.map = np.zeros((11, 11), dtype=int)
        self.received_targets: list[np.ndarray | None] = []

    def step(
        self,
        team: FakeTeam,
        step_num: int,
        *,
        external_goal_density: np.ndarray | None = None,
        update_legacy_gp: bool = True,
    ) -> float:
        self.step_count += 1
        assert not update_legacy_gp
        self.received_targets.append(external_goal_density)
        for agent in team.agents:
            agent.position = agent.position + np.array([0.01, 0.0])
            agent.history.append(agent.position.copy())
        return float(step_num)


class FakeGroundController:
    def __init__(self, team: FakeTeam) -> None:
        self.team = team
        self.step_count = 0
        axis = np.linspace(0.0, 10.0, 11)
        grid_x, grid_y = np.meshgrid(axis, axis)
        self.query_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
        self.integration_weights = np.ones(self.query_points.shape[0])
        self.received_densities: list[np.ndarray | None] = []
        self._density_source = "legacy_posthoc_fusion"

    @property
    def density_source(self) -> str:
        return self._density_source

    def step(
        self,
        step_num: int,
        hedac: FakeHedac,
        *,
        external_density: np.ndarray | None = None,
    ) -> None:
        del step_num, hedac
        self.step_count += 1
        self.received_densities.append(external_density)
        if external_density is not None:
            self._density_source = "multifidelity_high_posterior"
        for agent in self.team.agents:
            agent.position = agent.position + np.array([0.0, 0.01])
            agent.history.append(agent.position.copy())


def coordinator_config(
    low_period: float = 0.4,
    high_period: float = 0.2,
    update_period: float = 0.5,
) -> tuple[dict, dict]:
    aerial = {
        "simulation": {"random_seed": 12},
        "sensor": {"fov_depth": 2.0, "fov_degrees": 360.0},
        "gpr": {"obs_per_step": 1},
        "multifidelity": {
            "rho": 0.8,
            "aerial_sensor_period": low_period,
            "ground_sensor_period": high_period,
            "gp_update_period": update_period,
            "low_kernel": {"length_scale": 2.0, "variance": 1.0},
            "discrepancy_kernel": {"length_scale": 0.8, "variance": 0.3},
            "low_noise_variance": 0.01,
            "high_noise_variance": 0.001,
            "jitter": 1.0e-9,
            "retention": {
                "max_low_samples": 30,
                "max_high_samples": 30,
                "min_low_separation": 0.0,
                "min_high_separation": 0.0,
            },
            "density": {"normalization_tolerance": 1.0e-9},
            "sensor": {
                "low_fidelity_smoothing_sigma_cells": 1.0,
                "random_seed_offset": 50,
            },
        },
    }
    ground = {
        "sensor": {"fov_depth": 2.0, "fov_degrees": 90.0},
        "gpr": {"obs_per_step": 1},
    }
    return aerial, ground


def make_coordinator(
    low_period: float = 0.4,
    high_period: float = 0.2,
    update_period: float = 0.5,
    hyperparameter_optimization: dict | None = None,
):
    aerial, ground = coordinator_config(low_period, high_period, update_period)
    if hyperparameter_optimization is not None:
        aerial["multifidelity"]["hyperparameter_optimization"] = (
            hyperparameter_optimization
        )
    x_values = np.linspace(0.0, 10.0, 11)
    y_values = np.linspace(0.0, 10.0, 11)
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    query = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    high_field = np.add.outer(np.linspace(0.0, 1.0, 11), np.ones(11))
    return build_multifidelity_coordinator(
        aerial,
        ground,
        high_field,
        query,
        grid_x.shape,
        np.ones(query.shape[0]),
    )


def test_hyperparameter_optimization_config_reaches_estimator() -> None:
    coordinator = make_coordinator(
        hyperparameter_optimization={
            "enabled": True,
            "fit_interval_updates": 3,
            "min_samples": 7,
            "num_restarts": 2,
            "max_iterations": 45,
            "bounds": {
                "low_length_scale": [0.4, 8.0],
                "low_variance": [0.1, 4.0],
                "discrepancy_length_scale": [0.2, 3.0],
                "discrepancy_variance": [0.01, 1.5],
            },
        }
    )
    settings = coordinator.estimator.settings.hyperparameter_optimization
    assert settings.enabled
    assert settings.fit_interval_updates == 3
    assert settings.min_samples == 7
    assert settings.num_restarts == 2
    assert settings.max_iterations == 45
    assert settings.bounds.low_length_scale == (0.4, 8.0)
    assert settings.bounds.discrepancy_variance == (0.01, 1.5)


def test_coordinator_sensors_expose_no_sampling_mode_selector() -> None:
    aerial, ground = coordinator_config()
    x_values = np.linspace(0.0, 10.0, 11)
    grid_x, grid_y = np.meshgrid(x_values, x_values)
    query = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    coordinator = build_multifidelity_coordinator(
        aerial,
        ground,
        np.ones((11, 11)),
        query,
        grid_x.shape,
        np.ones(query.shape[0]),
    )
    assert not hasattr(coordinator.low_sensor, "sampling_pattern")
    assert not hasattr(coordinator.high_sensor, "sampling_pattern")


def teams() -> tuple[FakeTeam, FakeTeam]:
    return (
        FakeTeam(
            [
                FakeAgent(0, np.array([4.0, 4.0])),
                FakeAgent(1, np.array([6.0, 6.0])),
            ]
        ),
        FakeTeam(
            [
                FakeAgent(10, np.array([5.0, 5.0])),
                FakeAgent(11, np.array([7.0, 5.0])),
            ]
        ),
    )


def test_periodic_event_exact_times_and_noninteger_ratios_do_not_drift() -> None:
    event = PeriodicEvent(period=0.25)
    fired = []
    for simulation_time in np.arange(0.0, 1.01, 0.1):
        if event.is_due(float(simulation_time)):
            fired.append(round(float(simulation_time), 10))
            event.mark_fired()
    assert fired == [0.0, 0.3, 0.5, 0.8, 1.0]
    assert event.fire_count == 5
    assert event.next_fire_time == 1.25


def test_distinct_sensor_and_update_periods_fire_at_expected_times() -> None:
    coordinator = make_coordinator()
    aerial_team, ground_team = teams()
    fired: dict[CoordinatorEventType, list[float]] = {
        event_type: [] for event_type in CoordinatorEventType
    }
    for simulation_time in np.arange(0.0, 1.01, 0.1):
        timestamp = round(float(simulation_time), 10)
        reports = (
            coordinator.collect_low_if_due(timestamp, aerial_team),
            coordinator.collect_high_if_due(timestamp, ground_team),
            coordinator.update_if_due(timestamp),
        )
        for report in reports:
            if report.fired:
                fired[report.event_type].append(timestamp)
    assert fired[CoordinatorEventType.LOW_COLLECTION] == [0.0, 0.4, 0.8]
    assert fired[CoordinatorEventType.HIGH_COLLECTION] == [
        0.0,
        0.2,
        0.4,
        0.6,
        0.8,
        1.0,
    ]
    assert fired[CoordinatorEventType.ESTIMATOR_UPDATE] == [0.0, 0.5, 1.0]
    assert coordinator.estimator.version == 3
    assert coordinator.latest_posterior is not None


def test_controllers_continue_between_updates_and_aerial_uses_cached_target() -> None:
    coordinator = make_coordinator(update_period=0.5)
    aerial_team, ground_team = teams()
    hedac = FakeHedac()
    ground_controller = FakeGroundController(ground_team)
    simulation = CoupledSimulation(
        mode=EstimatorMode.MULTIFIDELITY,
        dt=0.1,
        aerial_team=aerial_team,
        hedac=hedac,
        ground_team=ground_team,
        ground_controller=ground_controller,
        coordinator=coordinator,
    )
    result = simulation.run(11)
    update_count = sum(
        report.fired
        for report in result.estimator_reports
        if report.event_type is CoordinatorEventType.ESTIMATOR_UPDATE
    )
    assert hedac.step_count == ground_controller.step_count == 11
    assert update_count == 3
    assert hedac.received_targets[0] is None
    assert all(target is not None for target in hedac.received_targets[1:])
    assert hedac.received_targets[1] is hedac.received_targets[2]
    assert all(density is not None for density in ground_controller.received_densities)
    assert (
        ground_controller.received_densities[1]
        is ground_controller.received_densities[2]
    )
    assert result.aerial_density_source == "multifidelity_high_posterior"
    assert result.ground_density_source == "multifidelity_high_posterior"
    assert result.final_posterior_version == 3


def test_first_posterior_is_cached_and_no_data_update_preserves_identity() -> None:
    coordinator = make_coordinator(
        low_period=1.0, high_period=1.0, update_period=0.2
    )
    aerial_team, ground_team = teams()
    coordinator.collect_low_if_due(0.0, aerial_team)
    coordinator.collect_high_if_due(0.0, ground_team)
    first_report = coordinator.update_if_due(0.0)
    first = coordinator.latest_posterior
    assert first_report.fired
    assert first is not None and first.timestamp == 0.0 and first.version == 1
    no_data = coordinator.update_if_due(0.2)
    assert no_data.update_report is not None
    assert no_data.update_report.status.value == "no_new_data"
    assert coordinator.latest_posterior is first
    assert coordinator.estimator.version == 1


def test_delayed_out_of_order_injection_remains_supported() -> None:
    coordinator = make_coordinator(update_period=1.0)
    aerial_team, ground_team = teams()
    coordinator.collect_low_if_due(0.0, aerial_team)
    coordinator.collect_high_if_due(0.0, ground_team)
    coordinator.update_if_due(0.0)
    coordinator.estimator.submit_many(
        [
            Observation(0.8, "late-0", (5.0, 5.0), 1.2, Fidelity.HIGH, 0.001),
            Observation(0.3, "late-1", (5.5, 5.0), 1.0, Fidelity.HIGH, 0.001),
        ]
    )
    report = coordinator.estimator.update(0.9)
    assert report.status.value == "updated"
    assert coordinator.estimator.high_sample_count >= 2


def test_one_estimator_serves_multiple_robots_and_runs_deterministically() -> None:
    def run() -> tuple[np.ndarray, np.ndarray, int]:
        coordinator = make_coordinator()
        aerial_team, ground_team = teams()
        assert coordinator.estimator is coordinator.estimator
        for simulation_time in np.arange(0.0, 1.01, 0.1):
            timestamp = round(float(simulation_time), 10)
            coordinator.collect_low_if_due(timestamp, aerial_team)
            coordinator.collect_high_if_due(timestamp, ground_team)
            coordinator.update_if_due(timestamp)
        snapshot = coordinator.latest_posterior
        assert snapshot is not None
        return snapshot.high_mean, snapshot.high_variance, snapshot.version

    first = run()
    second = run()
    np.testing.assert_array_equal(first[0], second[0])
    np.testing.assert_array_equal(first[1], second[1])
    assert first[2] == second[2]
