"""Three-vehicle SITL setup and diagnostic acceptance contracts."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from docker.deployment_settings import DeploymentSettings
from docker.deployment_readiness import evaluate_readiness


def manifest():
    return yaml.safe_load(Path("docker/scenarios/sitl_three_drone.yaml").read_text())


def test_sitl_config_covers_startup_drift_and_has_explicit_rates():
    settings = DeploymentSettings.from_manifest(manifest())
    assert settings.bounds[0][0] < -1
    assert settings.bounds[1][0] < -1
    assert settings.command_hz == 20
    assert settings.max_speed == 0.5


def test_rejects_invalid_settings():
    data = manifest()
    data["control"]["plan_expiry_s"] = None
    with pytest.raises(ValueError):
        DeploymentSettings.from_manifest(data)


def test_duplicate_vehicle_namespaces_are_rejected():
    data = manifest()
    data["vehicles"][1]["namespace"] = data["vehicles"][0]["namespace"]
    with pytest.raises(ValueError):
        DeploymentSettings.from_manifest(data)


def healthy():
    data = manifest()
    now = 10.0
    status = {
        "scenario_hash": "s",
        "timestamp": now,
        "posterior_version": 2,
        "low_sample_count": 3,
        "high_sample_count": 6,
    }
    frames = {
        "scenario_hash": "s",
        "timestamp": now,
        "expires_at": 10.75,
        "offsets": {v["id"]: [0, 0] for v in data["vehicles"]},
    }
    agents = {
        v["id"]: {
            "scenario_hash": "s",
            "timestamp": now,
            "registered": True,
            "pose_valid": True,
            "altitude_ok": True,
            "state": "HOLD",
            "commands_enabled": False,
        }
        for v in data["vehicles"]
    }
    poses = {
        v["id"]: {"scenario_hash": "s", "timestamp": now, "position": [0, 0]}
        for v in data["vehicles"]
    }
    plans = {
        v["id"]: {
            "scenario_hash": "s",
            "timestamp": now,
            "expires_at": 12,
            "vehicle_id": v["id"],
            "velocity": [0.1, 0],
        }
        for v in data["vehicles"]
    }
    return data, status, frames, agents, poses, plans


def test_readiness_requires_all_three_streams_and_both_fidelities():
    data, status, frames, agents, poses, plans = healthy()
    assert evaluate_readiness(data, "s", 10.1, status, frames, agents, poses, plans)[
        "ready"
    ]
    incomplete = deepcopy(agents)
    incomplete.pop("uav_3")
    result = evaluate_readiness(
        data, "s", 10.1, status, frames, incomplete, poses, plans
    )
    assert not result["ready"]
    assert any("uav_3" in reason for reason in result["reasons"])
    status["low_sample_count"] = 0
    assert not evaluate_readiness(
        data, "s", 10.1, status, frames, agents, poses, plans
    )["ready"]


def test_readiness_rejects_expired_offsets_and_wrong_altitude():
    data, status, frames, agents, poses, plans = healthy()
    frames["expires_at"] = 10
    agents["uav_2"]["altitude_ok"] = False
    result = evaluate_readiness(data, "s", 10.1, status, frames, agents, poses, plans)
    assert not result["ready"]
    assert len(result["reasons"]) >= 2


def test_readiness_rejects_prior_frame_session_in_plan():
    data, status, frames, agents, poses, plans = healthy()
    frames["session"] = "new"
    for table in (agents, poses, plans):
        for packet in table.values():
            packet["frame_session"] = "new"
    plans["uav_2"]["frame_session"] = "old"
    assert not evaluate_readiness(
        data, "s", 10.1, status, frames, agents, poses, plans
    )["ready"]


def test_shared_replacement_does_not_require_permanent_retained_fidelity_quotas():
    data, status, frames, agents, poses, plans = healthy()
    status["low_sample_count"] = 0
    status["sample_filter"] = dict(
        enabled=True,
        on_full="information",
        max_total_samples=1000,
        accepted_low=2,
        accepted_high=1002,
    )
    assert evaluate_readiness(data, "s", 10.1, status, frames, agents, poses, plans)[
        "ready"
    ]
    # Both fidelity streams must still have contributed before starting.
    status["sample_filter"]["accepted_low"] = 0
    assert not evaluate_readiness(
        data, "s", 10.1, status, frames, agents, poses, plans
    )["ready"]
