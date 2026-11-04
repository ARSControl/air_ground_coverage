"""Role speed configuration is shared by planners and readiness gates."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from docker.deployment_settings import DeploymentSettings
from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_readiness import evaluate_readiness


def manifest():
    return yaml.safe_load(Path("docker/scenarios/sitl_three_drone.yaml").read_text())


def test_active_aerial_speed_is_twice_ground_and_legacy_falls_back():
    data = manifest()
    settings = DeploymentSettings.from_manifest(data)
    assert settings.max_speed_for_role("aerial") == 1.0
    assert settings.max_speed_for_role("ground") == 0.5
    data["control"].pop("max_speed_by_role_mps", None)
    old = DeploymentSettings.from_manifest(data)
    assert old.max_speed_for_role("aerial") == old.max_speed_for_role("ground") == 0.5


@pytest.mark.parametrize(
    "limits",
    [
        {"aerial": 0},
        {"aerial": float("nan")},
        {"ground": -1},
        {"aerial": True},
        {"unknown": 1},
        [],
    ],
)
def test_invalid_role_speeds_are_rejected(limits):
    data = manifest()
    data["control"]["max_speed_by_role_mps"] = limits
    with pytest.raises(ValueError):
        DeploymentSettings.from_manifest(data)


def test_onboard_proposals_and_readiness_respect_each_role_limit():
    data = manifest()
    settings = DeploymentSettings.from_manifest(data)
    y, x = np.indices((21, 21))
    density = np.exp(-((x - 16) ** 2 + (y - 10) ** 2) / 4).ravel()
    target = dict(
        scenario_hash="s",
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
            scenario_hash="s",
            vehicle_id=v["id"],
            timestamp=10.0,
            position=[5.0 + i * 5, 10.0],
            heading=0.0,
        )
        for i, v in enumerate(data["vehicles"])
    }
    plans = {}
    for vehicle in data["vehicles"]:
        planner = OnboardPlanner(
            data,
            "s",
            vehicle["id"],
            max_speed=settings.max_speed_for_role(vehicle["role"]),
            max_age=2,
        )
        for pose in poses.values():
            planner.receive_pose(pose)
        plans[vehicle["id"]] = planner.plan(target, 10.1)
    assert np.linalg.norm(plans["uav_3"]["velocity"]) == pytest.approx(1.0)
    assert all(
        np.linalg.norm(plans[v]["velocity"]) <= 0.5 + 1e-9 for v in ("uav_1", "uav_2")
    )
    status = dict(
        scenario_hash="s",
        timestamp=10.0,
        posterior_version=1,
        low_sample_count=3,
        high_sample_count=3,
    )
    frames = dict(
        scenario_hash="s",
        timestamp=10.0,
        expires_at=11.0,
        offsets={v: [0, 0] for v in poses},
    )
    agents = {
        v: dict(
            scenario_hash="s",
            timestamp=10.0,
            registered=True,
            pose_valid=True,
            altitude_ok=True,
            state="HOLD",
        )
        for v in poses
    }
    assert evaluate_readiness(data, "s", 10.1, status, frames, agents, poses, plans)[
        "ready"
    ]
    invalid = deepcopy(plans)
    invalid["uav_1"]["velocity"] = [0.8, 0.0]
    assert not evaluate_readiness(
        data, "s", 10.1, status, frames, agents, poses, invalid
    )["ready"]
