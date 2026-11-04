"""SITL restores the small budgets and keeps rejected-sample updates alive."""

from pathlib import Path

import numpy as np
import yaml

from docker.deployment_runtime import DeploymentRuntime
from src.core.observations import Fidelity, Observation


def manifest():
    return yaml.safe_load(Path("docker/scenarios/sitl_three_drone.yaml").read_text())


def test_active_sitl_uses_paper_filter_and_shared_1000_cap():
    data = manifest()
    assert data["estimator"]["retention"]["max_total_samples"] == 1000
    assert "max_low_samples" not in data["estimator"]["retention"]
    assert "max_high_samples" not in data["estimator"]["retention"]
    assert data["estimator"]["retention"]["policy"] == "paper_epistemic"
    runtime = DeploymentRuntime(data)
    assert runtime.estimator.estimator.settings.sample_filter.enabled


def test_all_rejected_and_idle_updates_refresh_publication_without_refitting():
    data = manifest()
    data["estimator"]["retention"]["policy"] = "paper_epistemic"
    runtime = DeploymentRuntime(data)
    identity = runtime.scenario.scenario_hash
    for vehicle, fidelity in [("uav_1", Fidelity.HIGH), ("uav_3", Fidelity.LOW)]:
        runtime.estimator.submit_batch(
            identity,
            vehicle,
            "first",
            (Observation(0.0, vehicle, (15.0, 15.0), 1.0, fidelity, 1e-10),),
        )
    first = runtime.tick(0.0)
    snapshot = runtime.estimator.estimator.latest_posterior
    for vehicle, fidelity in [("uav_1", Fidelity.HIGH), ("uav_3", Fidelity.LOW)]:
        runtime.estimator.submit_batch(
            identity,
            vehicle,
            "redundant",
            (Observation(1.0, vehicle, (15.0, 15.0), 1.0, fidelity, 1e-10),),
        )
    refresh = runtime.tick(1.0)
    assert refresh is not None
    assert refresh["version"] == first["version"]
    assert refresh["timestamp"] == 1.0
    assert refresh["posterior_timestamp"] == 0.0
    assert refresh["model_updated"] is False
    np.testing.assert_array_equal(refresh["high_mean"], first["high_mean"])
    assert runtime.estimator.estimator.latest_posterior is snapshot
    assert runtime.estimator.estimator.pending_count == 0
    assert runtime.estimator.estimator.filter_diagnostics["redundant"] == 2
    assert runtime.tick(1.5) is None
    assert runtime.tick(2.0)["timestamp"] == 2.0


def test_filtered_high_observation_changes_onboard_aerial_target_and_command():
    from docker.deployment_onboard_planner import OnboardPlanner
    from docker.deployment_settings import DeploymentSettings

    data = manifest()
    runtime = DeploymentRuntime(data)
    identity = runtime.scenario.scenario_hash

    def sample(vehicle, timestamp):
        runtime.sample(
            dict(
                scenario_hash=identity,
                vehicle_id=vehicle,
                request_id=vehicle,
                timestamp=timestamp,
                position=[15.0, 15.0],
                heading=0.0,
            )
        )

    sample("uav_3", 0.0)
    before = runtime.tick(0.0)
    sample("uav_1", 0.5)
    after = runtime.tick(1.0)
    assert runtime.estimator.estimator.high_sample_count > 0
    assert not np.allclose(before["high_mean"], after["high_mean"])
    assert not np.allclose(before["high_variance"], after["high_variance"])
    assert not np.allclose(before["density"], after["density"])
    velocities = []
    for target in (before, after):
        settings = DeploymentSettings.from_manifest(data)
        planner = OnboardPlanner(
            data,
            identity,
            "uav_3",
            max_speed=settings.max_speed_for_role("aerial"),
            max_age=2.0,
        )
        planner.receive_pose(
            dict(
                scenario_hash=identity,
                vehicle_id="uav_3",
                timestamp=1.0,
                position=[15.0, 15.0],
                heading=0.0,
                frame_session="test",
            )
        )
        velocities.append(planner.plan(target, 1.0)["velocity"])
    assert not np.allclose(*velocities)


def test_shared_budget_configuration_rejects_mixed_or_invalid_caps():
    import pytest

    for cap in (0, True, 1.5):
        data = manifest()
        data["estimator"]["retention"]["max_total_samples"] = cap
        with pytest.raises((ValueError, TypeError)):
            DeploymentRuntime(data)
    data = manifest()
    data["estimator"]["retention"]["max_low_samples"] = 250
    with pytest.raises(ValueError, match="shared budget"):
        DeploymentRuntime(data)


def test_active_replacement_updates_high_target_and_aerial_command_at_capacity():
    from docker.deployment_onboard_planner import OnboardPlanner

    data = manifest()
    assert data["estimator"]["retention"]["on_full"] == "information"
    data["estimator"]["retention"]["max_total_samples"] = 3
    runtime = DeploymentRuntime(data)
    identity = runtime.scenario.scenario_hash
    for vehicle, batch in [
        (
            "uav_3",
            (
                Observation(0.0, "uav_3", (15.0, 15.0), 0.2, Fidelity.LOW, 0.04),
                Observation(0.0, "uav_3", (15.05, 15.0), 0.2, Fidelity.LOW, 0.04),
            ),
        ),
        (
            "uav_1",
            (Observation(0.0, "uav_1", (40.0, 40.0), 0.2, Fidelity.HIGH, 0.0001),),
        ),
    ]:
        runtime.estimator.submit_batch(identity, vehicle, "initial", batch)
    before = runtime.tick(0.0)
    runtime.estimator.submit_batch(
        identity,
        "uav_1",
        "replacement",
        (Observation(1.0, "uav_1", (15.0, 15.0), 1.0, Fidelity.HIGH, 0.0001),),
    )
    after = runtime.tick(1.0)
    assert after["model_updated"]
    assert after["sample_filter"]["replaced"] == 1
    assert after["sample_filter"]["evicted_low"] == 1
    assert after["sample_filter"]["retained_total"] == 3
    assert (
        after["sample_filter"]["last_replacement_objective_after"]
        < after["sample_filter"]["last_replacement_objective_before"]
    )
    assert not np.allclose(before["high_mean"], after["high_mean"])
    assert not np.allclose(before["density"], after["density"])
    velocities = []
    for target in (before, after):
        planner = OnboardPlanner(data, identity, "uav_3", max_speed=1.0, max_age=2.0)
        planner.receive_pose(
            dict(
                scenario_hash=identity,
                vehicle_id="uav_3",
                timestamp=1.0,
                position=[20.0, 15.0],
                heading=0.0,
                frame_session="test",
            )
        )
        velocities.append(planner.plan(target, 1.0)["velocity"])
    assert not np.allclose(*velocities)
