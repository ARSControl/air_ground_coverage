"""Onboard planners preserve centralized numerics and isolate role teams."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_ground_runtime import GroundPlannerRuntime


def fixture():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    y, x = np.indices((21, 21))
    density = np.exp(-((x - 15) ** 2 + (y - 10) ** 2) / 4).ravel()
    target = dict(
        scenario_hash="hash",
        timestamp=10.0,
        version=1,
        shape=[21, 21],
        query_points=np.column_stack((x.ravel(), y.ravel())).tolist(),
        density=density.tolist(),
        high_density=density.tolist(),
        integration_weights=np.ones(441).tolist(),
    )
    poses = {
        v["id"]: dict(
            scenario_hash="hash",
            vehicle_id=v["id"],
            timestamp=10.0,
            position=[5.0 + 5 * i, 10.0],
            heading=0.0,
            frame_session="session",
        )
        for i, v in enumerate(manifest["vehicles"])
    }
    return manifest, target, poses


def test_local_ground_outputs_match_team_planner_and_ignore_aerial():
    manifest, target, poses = fixture()
    ground = {k: poses[k] for k in ("uav_1", "uav_2")}
    expected = GroundPlannerRuntime("hash", ground, max_speed=0.5, max_age=2).plan(
        target, ground, 10.5
    )
    for i, vehicle in enumerate(ground):
        planner = OnboardPlanner(manifest, "hash", vehicle, max_speed=0.5, max_age=2)
        for packet in poses.values():
            planner.receive_pose(packet)
        assert set(planner.poses) == set(ground)
        assert planner.plan(target, 10.5) == expected[i]


def test_missing_stale_and_old_session_neighbor_rejected():
    manifest, target, poses = fixture()
    planner = OnboardPlanner(manifest, "hash", "uav_1", max_speed=0.5, max_age=2)
    planner.receive_pose(poses["uav_1"])
    with pytest.raises(ValueError):
        planner.plan(target, 10.5)
    peer = deepcopy(poses["uav_2"])
    peer["timestamp"] = 1.0
    planner.receive_pose(peer)
    with pytest.raises(ValueError):
        planner.plan(target, 10.5)
    peer.update(timestamp=10.0, frame_session="old")
    planner.receive_pose(peer)
    with pytest.raises(ValueError):
        planner.plan(target, 10.5)


def test_single_aerial_needs_only_own_pose_and_target():
    manifest, target, poses = fixture()
    planner = OnboardPlanner(manifest, "hash", "uav_3", max_speed=0.5, max_age=2)
    for packet in poses.values():
        planner.receive_pose(packet)
    assert set(planner.poses) == {"uav_3"}
    result = planner.plan(target, 10.5)
    assert result["vehicle_id"] == "uav_3"
    assert np.linalg.norm(result["velocity"]) <= 0.5 + 1e-9


def test_compose_has_no_shared_planners():
    services = yaml.safe_load(Path("docker/compose.yaml").read_text())["services"]
    assert "aerial-planner" not in services
    assert "ground-planner" not in services


def test_multiple_aerial_vehicles_rejected_explicitly():
    manifest, _, _ = fixture()
    extra = deepcopy(manifest["vehicles"][-1])
    extra["id"] = "uav_4"
    manifest["vehicles"].append(extra)
    with pytest.raises(ValueError, match="one aerial"):
        OnboardPlanner(manifest, "hash", "uav_3", max_speed=0.5, max_age=2)


def test_real_ground_observation_changes_onboard_aerial_proposal():
    from examples.plot_onboard_planning_milestone import outputs

    _, _, metrics = outputs()
    assert metrics["posterior_mean_max_abs_change"] > 0
    assert metrics["target_max_abs_change"] > 0
    assert metrics["aerial_velocity_change"] > 1e-8
    assert metrics["ground_velocity_max_abs_errors"] == [0.0, 0.0]
