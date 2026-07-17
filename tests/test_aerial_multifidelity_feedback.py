"""Causal and controller-level tests for multifidelity aerial feedback."""

from __future__ import annotations

import numpy as np
import pytest

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.core.observations import Fidelity, Observation
from src.models.agents import AgentTeam, DubinsAgent
from src.simulation import build_multifidelity_coordinator


def _aerial_config() -> dict:
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


def _ground_config() -> dict:
    return {
        "sensor": {"fov_depth": 2.0, "fov_degrees": 90.0},
        "gpr": {"obs_per_step": 1},
    }


def _feedback_case():
    query_axis = np.linspace(0.0, 8.0, 17)
    grid_x, grid_y = np.meshgrid(query_axis, query_axis)
    query_points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    high_field = np.exp(-((grid_x - 6.5) ** 2 + (grid_y - 4.0) ** 2) / 0.8)
    coordinator = build_multifidelity_coordinator(
        _aerial_config(),
        _ground_config(),
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
    low_report = coordinator.estimator.update(0.0)
    assert low_report.status.value == "updated"
    low_snapshot = coordinator.latest_posterior
    assert low_snapshot is not None

    controller_axis = np.arange(9, dtype=float)
    weights = np.ones((9, 9), dtype=float)
    mask = np.ones((9, 9), dtype=bool)
    low_target = coordinator.aerial_target(
        controller_axis, controller_axis, weights, mask
    )
    assert low_target is not None

    coordinator.estimator.submit(
        Observation(0.1, "ground-0", (6.5, 4.0), 4.0, Fidelity.HIGH, 0.0001)
    )
    high_report = coordinator.estimator.update(0.1)
    assert high_report.status.value == "updated"
    high_snapshot = coordinator.latest_posterior
    assert high_snapshot is not None
    high_target = coordinator.aerial_target(
        controller_axis, controller_axis, weights, mask
    )
    assert high_target is not None
    return coordinator, low_snapshot, high_snapshot, low_target, high_target


def _hedac_with_agent(goal: np.ndarray) -> tuple[HEDACAlgorithm, AgentTeam]:
    params = HEDACParams.from_dict(_aerial_config())
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
    return HEDACAlgorithm(params, map_loader, goal), AgentTeam([agent])


@pytest.mark.parametrize(
    "invalid",
    [
        np.ones((8, 9)),
        np.full((9, 9), np.nan),
        -np.ones((9, 9)),
        np.zeros((9, 9)),
    ],
)
def test_hedac_rejects_invalid_external_targets(invalid: np.ndarray) -> None:
    goal = np.ones((9, 9), dtype=float) / 81.0
    hedac, team = _hedac_with_agent(goal)
    with pytest.raises(ValueError):
        hedac.step(
            team,
            external_goal_density=invalid,
            update_legacy_gp=False,
        )


def test_default_hedac_step_keeps_legacy_gp_contract(monkeypatch) -> None:
    goal = np.ones((9, 9), dtype=float) / 81.0
    hedac, _ = _hedac_with_agent(goal)
    empty_team = AgentTeam([])
    calls = {"collect": 0, "update": 0}

    def collect(team):
        del team
        calls["collect"] += 1
        return np.empty((0, 3))

    def update(observations, step_num=0):
        del observations, step_num
        calls["update"] += 1

    monkeypatch.setattr(hedac, "collect_observations", collect)
    monkeypatch.setattr(hedac, "update_gp", update)
    hedac.step(empty_team)
    assert calls == {"collect": 1, "update": 1}

    hedac.step(empty_team, update_legacy_gp=False)
    assert calls == {"collect": 1, "update": 1}
    np.testing.assert_array_equal(hedac.current_goal_density, goal)


def test_high_observation_changes_posterior_and_cached_aerial_target() -> None:
    coordinator, before, after, target_before, target_after = _feedback_case()
    posterior_mean_change = np.linalg.norm(after.high_mean - before.high_mean)
    posterior_variance_change = np.linalg.norm(
        after.high_variance - before.high_variance
    )
    target_change = np.linalg.norm(target_after - target_before)

    assert after.high_sample_count == 1
    assert posterior_mean_change > 1.0
    assert posterior_variance_change > 0.1
    assert target_change > 0.005
    assert target_before.sum() == pytest.approx(1.0)
    assert target_after.sum() == pytest.approx(1.0)
    assert not target_before.flags.writeable
    assert coordinator.aerial_target(
        np.arange(9, dtype=float),
        np.arange(9, dtype=float),
        np.ones((9, 9)),
        np.ones((9, 9), dtype=bool),
    ) is target_after


def test_changed_multifidelity_target_changes_real_hedac_trajectory() -> None:
    _, _, _, target_before, target_after = _feedback_case()
    hedac_before, team_before = _hedac_with_agent(target_before)
    hedac_after, team_after = _hedac_with_agent(target_before)

    for step_num in range(8):
        hedac_before.step(
            team_before,
            step_num,
            external_goal_density=target_before,
            update_legacy_gp=False,
        )
        hedac_after.step(
            team_after,
            step_num,
            external_goal_density=target_after,
            update_legacy_gp=False,
        )

    trajectory_before = team_before.get_histories()[0]
    trajectory_after = team_after.get_histories()[0]
    assert np.linalg.norm(trajectory_after - trajectory_before) > 1.0e-4
    assert np.linalg.norm(
        hedac_after.heat_field - hedac_before.heat_field
    ) > 1.0e-4
