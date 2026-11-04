"""Deployment HEDAC planner consumes posterior targets without moving robots."""

import numpy as np
import pytest
import yaml
from pathlib import Path

from docker.deployment_hedac_planner import AerialHedacPlanner
from src.core.deployment_estimator import DeploymentEstimator
from src.core.deployment_scenario import build_scenario


def test_target_changes_aerial_velocity_and_never_moves_input_pose():
    y, x = np.indices((21, 21))
    left = np.exp(-((x - 4) ** 2 + (y - 10) ** 2) / 4)
    right = np.exp(-((x - 16) ** 2 + (y - 10) ** 2) / 4)
    pose = np.array([10.0, 10.0])
    a = AerialHedacPlanner((21, 21), 1.0, max_speed=1.0)
    b = AerialHedacPlanner((21, 21), 1.0, max_speed=1.0)
    v_left = a.step(pose, 0.0, left)
    v_right = b.step(pose, 0.0, right)
    np.testing.assert_array_equal(pose, [10, 10])
    assert not np.allclose(v_left, v_right)
    assert np.linalg.norm(v_left) <= 1.0 + 1e-9


def test_invalid_positions_and_target_rejected():
    planner = AerialHedacPlanner((21, 21), 1.0, max_speed=1.0)
    with pytest.raises(ValueError):
        planner.step((-1, 1), 0, np.ones((21, 21)))
    with pytest.raises(ValueError):
        planner.step((1, 1), 0, np.full((21, 21), np.nan))


def test_real_high_batch_changes_deployment_aerial_velocity():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=21)
    low = scenario.service.sample(scenario.scenario_hash, "uav_3", "a", 0, (15, 15), 0)
    estimator.submit_batch(scenario.scenario_hash, "uav_3", "a", low)
    estimator.update(0)
    before = estimator.aerial_target().reshape((21, 21))
    high = scenario.service.sample(
        scenario.scenario_hash, "uav_1", "a", 0.5, (15, 15), 0
    )
    estimator.submit_batch(scenario.scenario_hash, "uav_1", "a", high)
    estimator.update(1)
    after = estimator.aerial_target().reshape((21, 21))
    a = AerialHedacPlanner((21, 21), 2.5, max_speed=1)
    b = AerialHedacPlanner((21, 21), 2.5, max_speed=1)
    assert not np.allclose(a.step((15, 15), 0, before), b.step((15, 15), 0, after))
