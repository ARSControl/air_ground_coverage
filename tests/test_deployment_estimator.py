"""Integration of deployment observations and the central estimator."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from src.core.deployment_estimator import DeploymentEstimator
from src.core.deployment_scenario import build_scenario
from src.core.multifidelity_estimator import UpdateStatus


def test_high_observation_changes_central_posterior_after_scheduled_update():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=9)
    low = scenario.service.sample(scenario.scenario_hash, "uav_3", "1", 0, (15, 15), 0)
    estimator.submit_batch(scenario.scenario_hash, "uav_3", "1", low)
    assert estimator.estimator.latest_posterior is None
    assert estimator.update(0).status is UpdateStatus.UPDATED
    before = estimator.estimator.latest_posterior
    before_target = estimator.aerial_target()
    high = scenario.service.sample(
        scenario.scenario_hash, "uav_1", "1", 0.5, (15, 15), 0
    )
    estimator.submit_batch(scenario.scenario_hash, "uav_1", "1", high)
    assert estimator.update(0.5) is None
    assert estimator.update(1).status is UpdateStatus.UPDATED
    after = estimator.estimator.latest_posterior
    assert after.version == 2
    assert not np.allclose(before.high_mean, after.high_mean)
    assert not np.allclose(before.density, after.density)
    assert not np.allclose(before_target, estimator.aerial_target())


def test_duplicate_batches_are_not_resubmitted_and_conflicts_rejected():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=5)
    batch = scenario.service.sample(
        scenario.scenario_hash, "uav_3", "a", 0, (15, 15), 0
    )
    assert estimator.submit_batch(scenario.scenario_hash, "uav_3", "a", batch) == len(
        batch
    )
    assert estimator.submit_batch(scenario.scenario_hash, "uav_3", "a", batch) == 0
    with pytest.raises(ValueError):
        estimator.submit_batch(scenario.scenario_hash, "uav_3", "a", ())
    with pytest.raises(ValueError):
        estimator.submit_batch("wrong", "uav_3", "b", batch)


def test_future_observation_cannot_be_used_in_earlier_update():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=5)
    batch = scenario.service.sample(
        scenario.scenario_hash, "uav_3", "f", 10, (15, 15), 0
    )
    estimator.submit_batch(scenario.scenario_hash, "uav_3", "f", batch)
    with pytest.raises(ValueError, match="future"):
        estimator.update(9)
    assert estimator.estimator.latest_posterior is None
    assert estimator.update(10).status is UpdateStatus.UPDATED


def test_sitl_shared_budget_retains_more_than_previous_500_total():
    from src.core.observations import Fidelity, Observation

    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=5)
    for vehicle, fidelity in [("uav_3", Fidelity.LOW), ("uav_1", Fidelity.HIGH)]:
        batch = tuple(
            Observation(
                timestamp=0.0,
                robot_id=vehicle,
                position=(float(i % 20), float(i // 20)),
                value=float(np.sin(i / 30)),
                fidelity=fidelity,
                noise_variance=0.01,
            )
            for i in range(300)
        )
        estimator.submit_batch(scenario.scenario_hash, vehicle, "retention", batch)
    assert estimator.update(0.0).status is UpdateStatus.UPDATED
    snapshot = estimator.estimator.latest_posterior
    assert estimator.estimator.settings.total_retention == 1000
    assert snapshot.low_sample_count > 250
    assert snapshot.low_sample_count + snapshot.high_sample_count == 600
