"""Read-only telemetry to sampling-request contract."""

import pytest

from docker.deployment_agent_gate import SamplingGate


def test_gate_uses_verified_offset_and_rejects_stale_pose():
    gate = SamplingGate("uav_1", "session-a", (0, 0), max_pose_age=1)
    gate.set_scenario("hash")
    request = gate.request(10.2, 10, (3, 4), 0.5, valid=True)
    assert request["position"] == [3, 4]
    assert request["heading"] == 0.5
    assert request["request_id"] == "session-a:0"
    assert gate.request(12, 10, (3, 4), 0.5, valid=True) is None


def test_gate_requires_scenario_offset_and_valid_finite_telemetry():
    gate = SamplingGate("uav_2", "session-b", None, max_pose_age=1)
    gate.set_scenario("hash")
    assert gate.request(2, 2, (3, 4), 0, valid=True) is None
    gate.set_offset((5, 0))
    assert gate.request(2, 2, (3, 4), 0, valid=False) is None
    assert gate.request(2, 2.1, (3, 4), 0, valid=True) is None
    assert gate.request(2, 2, (float("nan"), 4), 0, valid=True) is None
    assert gate.request(2, 2, (3, 4), 0, valid=True)["position"] == [8, 4]
    gate.clear_offset()
    assert gate.request(2, 2, (3, 4), 0, valid=True) is None
    with pytest.raises(ValueError):
        gate.set_offset((float("inf"), 0))
