"""Ground-role planner rejects incomplete or stale team state."""

import pytest

from docker.deployment_ground_runtime import GroundPlannerRuntime


def data():
    target = {
        "scenario_hash": "hash",
        "timestamp": 10.0,
        "version": 1,
        "query_points": [[0, 0], [1, 0], [2, 0]],
        "high_density": [1, 2, 1],
        "integration_weights": [1, 1, 1],
    }
    poses = {
        "a": {
            "scenario_hash": "hash",
            "vehicle_id": "a",
            "timestamp": 10.0,
            "position": [0, 0],
            "heading": 0.0,
        },
        "b": {
            "scenario_hash": "hash",
            "vehicle_id": "b",
            "timestamp": 10.0,
            "position": [2, 0],
            "heading": 3.14,
        },
    }
    return target, poses


def test_both_ground_roles_get_expiring_proposals():
    runtime = GroundPlannerRuntime("hash", ["a", "b"], max_speed=0.5, max_age=2)
    target, poses = data()
    plans = runtime.plan(target, poses, 10.5)
    assert [p["vehicle_id"] for p in plans] == ["a", "b"]
    assert all(p["expires_at"] == 12.5 for p in plans)


def test_incomplete_or_stale_team_is_rejected():
    runtime = GroundPlannerRuntime("hash", ["a", "b"], max_speed=0.5, max_age=2)
    target, poses = data()
    with pytest.raises(ValueError):
        runtime.plan(target, {"a": poses["a"]}, 10.5)
    poses["b"]["timestamp"] = 1
    with pytest.raises(ValueError):
        runtime.plan(target, poses, 10.5)


def test_missing_team_diagnostic_is_distinct_from_wrong_scenario():
    import pytest
    from docker.deployment_ground_runtime import GroundPlannerRuntime
    runtime = GroundPlannerRuntime('expected', ['uav_1', 'uav_2'], max_speed=.5, max_age=2)
    with pytest.raises(ValueError, match='missing team poses: uav_2'):
        runtime.plan({'scenario_hash': 'expected'}, {'uav_1': {}}, 1.)
    with pytest.raises(ValueError, match='scenario mismatch'):
        runtime.plan({'scenario_hash': 'wrong'}, {}, 1.)
