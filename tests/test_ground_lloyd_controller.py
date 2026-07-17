"""Tests for selectable density-weighted Lloyd ground coverage."""

from __future__ import annotations

import numpy as np
import pytest

from src.core.base import HEDACParams
from src.coupled_config import load_coupled_configuration
from src.coupled_simulation import (
    _GroundLloydController,
    _ground_controller_type,
    build_coupled_simulation,
    compute_weighted_voronoi_centroids,
)
from src.models.agents import AgentTeam, UnicycleAgent


def _params(**lloyd_overrides) -> HEDACParams:
    lloyd = {
        "position_gain": 1.0,
        "heading_gain": 2.0,
        "centroid_tolerance": 0.01,
        "max_linear_velocity": 1.5,
        "max_angular_velocity": 1.0,
        **lloyd_overrides,
    }
    return HEDACParams.from_dict(
        {
            "controller": {"type": "lloyd"},
            "simulation": {"num_agents": 1, "dt": 0.1},
            "agents": {
                "model_type": "unicycle",
                "max_velocity": 2.0,
                "max_angular_velocity": 2.0,
                "dt_agent": 0.1,
            },
            "sensor": {"fov_depth": 2.0, "fov_degrees": 360.0},
            "gpr": {"obs_per_step": 1},
            "map": {"local_grid_points": 11},
            "lloyd": lloyd,
        }
    )


def _team(position=(1.0, 1.0), heading=0.0) -> AgentTeam:
    return AgentTeam(
        [
            UnicycleAgent(
                np.asarray(position, dtype=float),
                heading,
                max_v=2.0,
                max_omega=2.0,
                dt=0.1,
            )
        ]
    )


def test_weighted_centroids_use_density_masks_and_quadrature_weights() -> None:
    points = np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0], [2.0, 2.0]])
    density = np.array([1.0, 1.0, 2.0, 4.0])
    masks = np.array([[1, 1, 0, 0], [0, 0, 1, 1]], dtype=bool)
    quadrature = np.array([1.0, 3.0, 2.0, 1.0])
    fallback = np.array([[9.0, 9.0], [8.0, 8.0]])

    centroids, masses = compute_weighted_voronoi_centroids(
        points, density, masks, quadrature, fallback
    )

    np.testing.assert_allclose(centroids, [[1.5, 0.0], [1.0, 2.0]])
    np.testing.assert_allclose(masses, [4.0, 8.0])
    assert not centroids.flags.writeable
    assert not masses.flags.writeable


def test_zero_mass_cell_holds_its_current_position() -> None:
    points = np.array([[0.0, 0.0], [1.0, 0.0]])
    centroids, masses = compute_weighted_voronoi_centroids(
        points,
        np.zeros(2),
        np.ones((1, 2), dtype=bool),
        np.ones(2),
        np.array([[0.25, 0.75]]),
    )
    np.testing.assert_array_equal(centroids, [[0.25, 0.75]])
    np.testing.assert_array_equal(masses, [0.0])


def test_lloyd_unicycle_moves_toward_weighted_centroid_with_bounded_control() -> None:
    params = _params()
    team = _team()
    controller = _GroundLloydController(
        params, np.zeros((10, 10)), team, np.zeros((10, 10))
    )
    density = np.zeros(controller.query_points.shape[0])
    target_index = np.argmin(
        np.linalg.norm(controller.query_points - np.array([8.0, 1.0]), axis=1)
    )
    density[target_index] = 1.0

    controller.step(0, object(), external_density=density)

    np.testing.assert_allclose(controller.last_centroids, [[8.0, 1.0]])
    assert controller.last_controls[0, 0] == pytest.approx(1.5)
    assert controller.last_controls[0, 1] == pytest.approx(0.0)
    assert team.agents[0].position[0] > 1.0
    assert controller.density_source == "multifidelity_high_posterior"


def test_lloyd_invalid_external_density_is_transactional() -> None:
    controller = _GroundLloydController(
        _params(), np.zeros((10, 10)), _team(), np.zeros((10, 10))
    )
    preserved_position = controller.team.agents[0].position.copy()
    with pytest.raises(ValueError, match="finite"):
        controller.step(
            0,
            object(),
            external_density=np.full(controller.query_points.shape[0], np.nan),
        )
    np.testing.assert_array_equal(controller.team.agents[0].position, preserved_position)
    np.testing.assert_array_equal(controller.last_density, 0.0)
    assert controller.density_source == "legacy_posthoc_fusion"


@pytest.mark.parametrize("value", ["unknown", "", 3, None])
def test_controller_selection_rejects_invalid_values(value) -> None:
    params = _params()
    params._config["controller"]["type"] = value
    with pytest.raises((TypeError, ValueError), match="controller.type"):
        _ground_controller_type(params)


def test_coupled_builder_selects_lloyd_and_uses_central_density() -> None:
    configuration = load_coupled_configuration(
        "configs/multifidelity_plot_smoke.yaml"
    )
    configuration.ground._config["controller"] = {"type": "lloyd"}
    configuration.ground._config["lloyd"] = {
        "position_gain": 1.0,
        "heading_gain": 2.0,
        "centroid_tolerance": 0.01,
        "max_linear_velocity": 1.0,
        "max_angular_velocity": 1.0,
    }

    simulation = build_coupled_simulation(
        configuration.aerial, configuration.ground, seed=77
    )
    result = simulation.run(2)
    snapshot = simulation.latest_posterior

    assert isinstance(simulation.ground_controller, _GroundLloydController)
    assert snapshot is not None
    assert result.ground_density_source == "multifidelity_high_posterior"
    np.testing.assert_array_equal(
        simulation.ground_controller.last_density, snapshot.density
    )
    assert simulation.ground_controller.gp.n_samples == 0
    assert result.ground_trajectories[0].shape[0] == 3
