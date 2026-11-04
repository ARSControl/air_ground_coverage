"""Fail-closed input checks for the deployment planner."""

import numpy as np
import pytest

from docker.deployment_planner_runtime import PlannerRuntime


def target():
    y, x = np.indices((21, 21))
    density = np.exp(-((x - 15) ** 2 + (y - 10) ** 2) / 4)
    return {
        "scenario_hash": "hash",
        "version": 1,
        "timestamp": 10.0,
        "shape": [21, 21],
        "query_points": np.column_stack((x.ravel(), y.ravel())).tolist(),
        "density": density.ravel().tolist(),
    }


def pose():
    return {
        "scenario_hash": "hash",
        "vehicle_id": "uav_3",
        "timestamp": 10.0,
        "position": [10.0, 10.0],
        "heading": 0.0,
    }


def test_planner_returns_bounded_plan_and_rejects_stale_data():
    planner = PlannerRuntime("hash", "uav_3", max_speed=1, max_age=2)
    result = planner.plan(target(), pose(), 10.5)
    assert result["vehicle_id"] == "uav_3"
    assert result["posterior_version"] == 1
    assert np.linalg.norm(result["velocity"]) <= 1.0 + 1e-9
    with pytest.raises(ValueError, match="stale"):
        planner.plan(target(), pose(), 13)


def test_planner_rejects_mismatched_frames_and_irregular_grid():
    planner = PlannerRuntime("hash", "uav_3", max_speed=1, max_age=2)
    bad = pose()
    bad["scenario_hash"] = "other"
    with pytest.raises(ValueError):
        planner.plan(target(), bad, 10.5)
    bad_target = target()
    bad_target["query_points"][7][0] += 0.3
    with pytest.raises(ValueError):
        planner.plan(bad_target, pose(), 10.5)
