"""Transport-independent behavior used by the ROS central service."""

from pathlib import Path

import pytest
import yaml

from docker.deployment_runtime import DeploymentRuntime


def runtime():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    return DeploymentRuntime(manifest, grid_points=9)


def test_sampling_updates_and_idempotent_retry():
    service = runtime()
    req = {
        "scenario_hash": service.scenario.scenario_hash,
        "vehicle_id": "uav_3",
        "request_id": "a",
        "timestamp": 1.0,
        "position": [15.0, 15.0],
        "heading": 0.0,
    }
    first = service.sample(req)
    assert first["count"] > 0
    assert len(first["observations"]) == first["count"]
    assert all(o["fidelity"] == "low" for o in first["observations"])
    assert service.sample(req)["observations"] == first["observations"]
    assert service.sample(req)["submitted"] == 0
    posterior = service.tick(1.0)
    assert posterior["version"] == 1
    assert len(posterior["high_density"]) == len(posterior["integration_weights"])
    assert service.tick(1.5) is None
    assert service.tick(2.0) is None


def test_reject_wrong_scenario_and_future_update():
    service = runtime()
    request = {
        "scenario_hash": "wrong",
        "vehicle_id": "uav_1",
        "request_id": "x",
        "timestamp": 10.0,
        "position": [15.0, 15.0],
        "heading": 0.0,
    }
    with pytest.raises(ValueError):
        service.sample(request)
    request["scenario_hash"] = service.scenario.scenario_hash
    service.sample(request)
    with pytest.raises(ValueError, match="future"):
        service.tick(9.0)
