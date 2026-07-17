"""Ground-controller integration tests for the central posterior density."""

from __future__ import annotations

import casadi as ca
import numpy as np
import pytest

from src.core import costFunctions
from src.core.observations import Fidelity, Observation
from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import _LegacyGroundMPC, build_coupled_simulation
from src.simulation import (
    build_ground_weight_vectors,
    build_multifidelity_coordinator,
)


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


def _ground_config() -> dict:
    return {
        "sensor": {"fov_depth": 2.0, "fov_degrees": 90.0},
        "gpr": {"obs_per_step": 1},
    }


def _coordinator_case():
    axis = np.linspace(0.0, 8.0, 17)
    grid_x, grid_y = np.meshgrid(axis, axis)
    points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    high_field = np.exp(-((grid_x - 6.5) ** 2 + (grid_y - 4.0) ** 2) / 0.8)
    coordinator = build_multifidelity_coordinator(
        _aerial_config(),
        _ground_config(),
        high_field,
        points,
        grid_x.shape,
        np.ones(points.shape[0]),
        seed=31,
    )
    return coordinator, points, grid_x, grid_y


def _publish_low_only(coordinator) -> None:
    observations = []
    for index, (x, y) in enumerate(
        (x, y) for y in (1.0, 3.0, 5.0, 7.0) for x in (1.0, 3.0, 5.0, 7.0)
    ):
        value = float(np.exp(-((x - 2.0) ** 2 + (y - 4.0) ** 2) / 10.0))
        observations.append(
            Observation(0.0, f"low-{index}", (x, y), value, Fidelity.LOW, 0.01)
        )
    coordinator.estimator.submit_many(observations)
    report = coordinator.estimator.update(0.0)
    assert report.status.value == "updated"


def test_ground_weight_vectors_are_exact_voronoi_products() -> None:
    density = np.array([0.1, 0.2, 0.3, 0.4])
    masks = np.array(
        [[True, True, False, False], [False, False, True, True]], dtype=bool
    )
    vectors = build_ground_weight_vectors(density, masks)
    assert len(vectors) == 2
    np.testing.assert_array_equal(vectors[0], np.array([0.1, 0.2, 0.0, 0.0]))
    np.testing.assert_array_equal(vectors[1], np.array([0.0, 0.0, 0.3, 0.4]))


@pytest.mark.parametrize(
    ("density", "masks"),
    [
        (np.array([[1.0, 2.0]]), np.ones((1, 2))),
        (np.array([1.0, np.nan]), np.ones((1, 2))),
        (np.array([1.0, -1.0]), np.ones((1, 2))),
        (np.array([1.0, 2.0]), np.ones(2)),
        (np.array([1.0, 2.0]), np.ones((1, 3))),
        (np.array([1.0, 2.0]), np.array([[1.0, np.inf]])),
        (np.array([1.0, 2.0]), np.array([[1.0, -1.0]])),
    ],
)
def test_ground_weight_vectors_reject_invalid_inputs(
    density: np.ndarray, masks: np.ndarray
) -> None:
    with pytest.raises(ValueError):
        build_ground_weight_vectors(density, masks)


def test_ground_density_fallback_cache_identity_and_transactional_validation() -> None:
    coordinator, points, _, _ = _coordinator_case()
    assert coordinator.ground_density(points) is None
    assert coordinator.ground_density_version == 0

    _publish_low_only(coordinator)
    snapshot = coordinator.latest_posterior
    assert snapshot is not None
    density = coordinator.ground_density(points, snapshot.integration_weights)
    assert density is snapshot.density
    assert coordinator.ground_density_version == snapshot.version == 1
    assert coordinator.ground_density(points, snapshot.integration_weights) is density

    bad_points = points.copy()
    bad_points[0, 0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        coordinator.ground_density(bad_points, snapshot.integration_weights)
    assert coordinator.ground_density(points, snapshot.integration_weights) is density
    assert coordinator.ground_density_version == 1


def test_local_high_correction_changes_ground_weights_and_coverage_cost() -> None:
    coordinator, points, grid_x, grid_y = _coordinator_case()
    _publish_low_only(coordinator)
    before_snapshot = coordinator.latest_posterior
    assert before_snapshot is not None
    before = coordinator.ground_density(points, before_snapshot.integration_weights)
    assert before is not None

    coordinator.estimator.submit(
        Observation(0.1, "ground-0", (6.5, 4.0), 4.0, Fidelity.HIGH, 0.0001)
    )
    report = coordinator.estimator.update(0.1)
    assert report.status.value == "updated"
    after_snapshot = coordinator.latest_posterior
    assert after_snapshot is not None
    after = coordinator.ground_density(points, after_snapshot.integration_weights)
    assert after is after_snapshot.density

    corrected_region = (grid_x.ravel() >= 5.5) & (
        np.abs(grid_y.ravel() - 4.0) <= 1.5
    )
    masks = np.vstack((corrected_region, ~corrected_region))
    before_weights = build_ground_weight_vectors(before, masks)[0]
    after_weights = build_ground_weight_vectors(after, masks)[0]
    assert after_weights.sum() > before_weights.sum()
    assert np.linalg.norm(after_weights - before_weights) > 0.01

    position = ca.SX.sym("position", 2)
    weights = ca.SX.sym("weights", points.shape[0])
    objective = ca.Function(
        "ground_feedback_coverage_cost",
        [position, weights],
        [costFunctions.coverage_cost(position, ca.DM(points), weights)],
    )
    # At the corrected HIGH location, adding local importance reduces the
    # squared-distance coverage cost, as expected.
    ground_position = np.array([6.5, 4.0])
    before_cost = float(objective(ground_position, before))
    after_cost = float(objective(ground_position, after))
    assert after_cost < before_cost

    controller_axis = np.arange(9, dtype=float)
    coordinator.aerial_target(
        controller_axis,
        controller_axis,
        np.ones((9, 9)),
        np.ones((9, 9), dtype=bool),
    )
    assert coordinator.aerial_target_version == after_snapshot.version
    assert coordinator.ground_density_version == after_snapshot.version


def test_real_ground_mpc_receives_exact_multifidelity_density() -> None:
    configuration = load_coupled_configuration(
        "configs/multifidelity_plot_smoke.yaml"
    )
    aerial = configuration.aerial
    ground = configuration.ground
    simulation = build_coupled_simulation(aerial, ground, seed=77)
    result = simulation.run(2)
    snapshot = simulation.latest_posterior
    controller = simulation.ground_controller

    assert snapshot is not None
    assert result.ground_density_source == "multifidelity_high_posterior"
    assert controller.density_source == "multifidelity_high_posterior"
    np.testing.assert_array_equal(controller.last_density, snapshot.density)
    assert controller.gp.n_samples == 0
    assert len(result.ground_trajectories) == 1
    assert result.ground_trajectories[0].shape[0] == 3


def test_legacy_fusion_values_remain_exact_and_invalid_external_is_transactional() -> None:
    class PredictionModel:
        def predict(self, points, return_std=True):
            assert return_std and points.shape == (2, 2)
            return np.array([0.2, 0.8]), np.array([1.0, 2.0])

    class GroundGP:
        def collect_observations(self, team, field):
            del team, field
            return np.array([[0.0, 0.0, 1.0]])

        def update_gp(self, observations, step_num):
            assert observations.shape == (1, 3) and step_num == 0

        def predict(self, points, return_std=True):
            assert return_std and points.shape == (2, 2)
            return np.array([0.9, 0.1]), np.array([2.0, 1.0])

    class Agent:
        position = np.array([0.0, 0.0])
        theta = 0.0

        def step(self, velocity, angular_velocity):
            self.last_control = (velocity, angular_velocity)

    class Team:
        def __init__(self):
            self.agents = [Agent()]

        def get_states(self):
            return np.array([[0.0, 0.0, 0.0]])

    class SolutionVector:
        def full(self):
            return np.zeros((2, 1))

    class Solver:
        def __init__(self):
            self.calls = 0

        def __call__(self, **kwargs):
            assert kwargs["p"].shape == (5,)
            self.calls += 1
            return {"x": SolutionVector()}

    controller = object.__new__(_LegacyGroundMPC)
    controller.query_points = np.array([[0.0, 0.0], [1.0, 0.0]])
    controller.integration_weights = np.ones(2)
    controller.team = Team()
    controller.high_field = np.ones((2, 2))
    controller.gp = GroundGP()
    controller._u_previous = np.zeros((1, 2))
    controller._solver = Solver()
    controller._bounds = (
        np.full(2, -1.0),
        np.full(2, 1.0),
        np.full(4, -np.inf),
        np.zeros(4),
    )
    controller.last_density = np.zeros(2)
    controller._density_source = "legacy_posthoc_fusion"
    hedac = type("Hedac", (), {"gpr_model": PredictionModel()})()

    controller.step(0, hedac)
    aerial_std = np.array([1.0, 2.0]) / (2.0 + 1.0e-10)
    ground_std = np.array([2.0, 1.0]) / (2.0 + 1.0e-10)
    denominator = 1.0 / (ground_std + 1.0e-10) + 1.0 / (
        aerial_std + 1.0e-10
    )
    expected = (
        (1.0 / (aerial_std + 1.0e-10)) / denominator
    ) * np.array([0.2, 0.8]) + (
        (1.0 / (ground_std + 1.0e-10)) / denominator
    ) * np.array([0.9, 0.1])
    np.testing.assert_array_equal(controller.last_density, expected)
    assert controller.density_source == "legacy_posthoc_fusion"
    assert controller._solver.calls == 1

    preserved = controller.last_density.copy()
    with pytest.raises(ValueError, match="finite"):
        controller.step(1, hedac, external_density=np.array([np.nan, 1.0]))
    np.testing.assert_array_equal(controller.last_density, preserved)
    assert controller.density_source == "legacy_posthoc_fusion"
    assert controller._solver.calls == 1
