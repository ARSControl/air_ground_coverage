"""Regression smoke tests for legacy mode and the unchanged HEDAC path."""

from __future__ import annotations

import numpy as np

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.coupled_simulation import CoupledSimulation, build_coupled_simulation
from src.models.agents import AgentTeam, DoubleIntegratorAgent
from src.simulation import EstimatorMode, parse_estimator_mode


class CountingGroundController:
    def __init__(self) -> None:
        self.steps = 0

    @property
    def density_source(self) -> str:
        return "legacy_posthoc_fusion"

    def step(self, step_num: int, hedac: HEDACAlgorithm) -> None:
        del step_num, hedac
        self.steps += 1


def tiny_params() -> HEDACParams:
    return HEDACParams.from_dict(
        {
            "simulation": {"num_steps": 2, "num_agents": 1, "dt": 0.1},
            "heat_equation": {
                "alpha": 0.1,
                "source_strength": 1.0,
                "beta": 0.01,
                "local_cooling": 0.1,
            },
            "agents": {
                "model_type": "double_integrator",
                "max_velocity": 1.0,
                "max_acceleration": 0.5,
                "max_angular_velocity": 0.785,
                "max_angular_acceleration": 0.393,
                "dt_agent": 0.1,
                "agent_radius": 1.0,
                "min_kernel_val": 0.01,
            },
            "sensor": {"fov_degrees": 360.0, "fov_depth": 2.0},
            "multi_agent": {"sensing_range": 3.0, "min_safe_distance": 0.5},
            "gpr": {
                "obs_per_step": 2,
                "obs_noise_std": 0.0,
                "min_samples": 100,
                "use_filter": False,
                "fit_interval": 10,
            },
            "map": {"size": [9, 9], "resolution": 1.0},
            "visualization": {"gp_debug": False},
        }
    )


def tiny_multifidelity_params() -> HEDACParams:
    config = tiny_params().to_dict()
    config["estimator_mode"] = "multifidelity"
    config["simulation"]["random_seed"] = 77
    config["multifidelity"] = {
        "rho": 0.8,
        "gp_update_period": 0.2,
        "aerial_sensor_period": 0.2,
        "ground_sensor_period": 0.1,
        "low_kernel": {"length_scale": 2.0, "variance": 1.0},
        "discrepancy_kernel": {"length_scale": 0.8, "variance": 0.3},
        "low_noise_variance": 0.01,
        "high_noise_variance": 0.001,
        "jitter": 1.0e-9,
        "retention": {
            "max_low_samples": 20,
            "max_high_samples": 20,
            "min_low_separation": 0.0,
            "min_high_separation": 0.0,
        },
        "aerial_target": {"lambda_interest": 1.0, "lambda_uncertainty": 0.25},
        "density": {"normalization_tolerance": 1.0e-9},
        "sensor": {
            "low_fidelity_smoothing_sigma_cells": 1.0,
            "random_seed_offset": 100,
        },
    }
    return HEDACParams.from_dict(config)


def tiny_ground_params() -> HEDACParams:
    return HEDACParams.from_dict(
        {
            "simulation": {"num_agents": 0},
            "agents": {
                "model_type": "unicycle",
                "max_velocity": 1.0,
                "max_acceleration": 0.5,
                "max_angular_velocity": 1.0,
                "dt_agent": 0.1,
            },
            "sensor": {"fov_degrees": 90.0, "fov_depth": 2.0},
            "gpr": {"obs_per_step": 1},
            "map": {"resolution": 1.0, "local_grid_points": 5},
        }
    )


def build_tiny_legacy_path() -> tuple[AgentTeam, HEDACAlgorithm]:
    params = tiny_params()
    map_loader = MapLoader(size=(9, 9), resolution=1.0)
    map_array = map_loader.load()
    grid_x, grid_y = np.meshgrid(np.arange(9), np.arange(9))
    goal = np.exp(-((grid_x - 6.0) ** 2 + (grid_y - 4.0) ** 2) / 8.0)
    goal *= map_array == 0
    goal /= goal.sum()
    agent = DoubleIntegratorAgent(
        x0=np.array([2.0, 4.0]),
        theta0=0.0,
        max_dx=1.0,
        max_ddx=0.5,
        dt=0.1,
        agent_id=0,
        observations_range=2,
        observations_count=2,
    )
    return AgentTeam([agent]), HEDACAlgorithm(params, map_loader, goal)


def test_missing_and_explicit_legacy_modes_resolve_identically() -> None:
    assert parse_estimator_mode({}) is EstimatorMode.LEGACY
    assert parse_estimator_mode({"estimator_mode": "legacy"}) is EstimatorMode.LEGACY
    assert (
        parse_estimator_mode({"estimator_mode": "multifidelity"})
        is EstimatorMode.MULTIFIDELITY
    )


def test_short_standalone_hedac_path_is_deterministic() -> None:
    def run() -> tuple[np.ndarray, np.ndarray]:
        np.random.seed(77)
        team, hedac = build_tiny_legacy_path()
        result = hedac.run(team, num_steps=2, verbose=False)
        return result["ergodic_metrics"], result["trajectories"][0]

    first_metrics, first_trajectory = run()
    second_metrics, second_trajectory = run()
    assert first_metrics.shape == (2,)
    assert first_trajectory.shape == (3, 2)
    np.testing.assert_array_equal(first_metrics, second_metrics)
    np.testing.assert_array_equal(first_trajectory, second_trajectory)


def test_legacy_coupled_branch_constructs_no_coordinator() -> None:
    np.random.seed(77)
    aerial_team, hedac = build_tiny_legacy_path()
    ground_controller = CountingGroundController()
    simulation = CoupledSimulation(
        mode=EstimatorMode.LEGACY,
        dt=0.1,
        aerial_team=aerial_team,
        hedac=hedac,
        ground_team=AgentTeam([]),
        ground_controller=ground_controller,
        coordinator=None,
    )
    result = simulation.run(2)
    assert simulation.coordinator is None
    assert simulation.latest_posterior is None
    assert result.final_posterior_version == 0
    assert result.estimator_reports == ()
    assert ground_controller.steps == 2
    assert result.aerial_density_source == "legacy_aerial_gp"
    assert result.ground_density_source == "legacy_posthoc_fusion"


def test_concrete_multifidelity_builder_runs_with_aerial_feedback() -> None:
    def run() -> tuple[np.ndarray, np.ndarray, int, str, str]:
        simulation = build_coupled_simulation(
            tiny_multifidelity_params(), tiny_ground_params(), seed=77
        )
        result = simulation.run(2)
        snapshot = simulation.latest_posterior
        assert simulation.coordinator is not None
        assert snapshot is not None
        return (
            result.aerial_trajectories[0],
            snapshot.high_mean,
            result.final_posterior_version,
            result.aerial_density_source,
            result.ground_density_source,
        )

    first = run()
    second = run()
    np.testing.assert_array_equal(first[0], second[0])
    np.testing.assert_array_equal(first[1], second[1])
    assert first[2:] == second[2:]
    assert first[2] == 1
    assert first[3:] == (
        "multifidelity_high_posterior",
        "multifidelity_high_posterior",
    )

    different_seed_simulation = build_coupled_simulation(
        tiny_multifidelity_params(), tiny_ground_params(), seed=78
    )
    different_seed_result = different_seed_simulation.run(2)
    assert not np.array_equal(first[0], different_seed_result.aerial_trajectories[0])


def test_concrete_legacy_builder_runs_without_constructing_coordinator() -> None:
    simulation = build_coupled_simulation(tiny_params(), tiny_ground_params(), seed=77)
    result = simulation.run(2)
    assert simulation.mode is EstimatorMode.LEGACY
    assert simulation.coordinator is None
    assert simulation.latest_posterior is None
    assert result.final_posterior_version == 0
    assert result.estimator_reports == ()
