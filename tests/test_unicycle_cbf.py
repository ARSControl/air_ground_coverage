"""Ground-role unicycle speed filter: feasible and infeasible cases."""

import numpy as np
import pytest

from src.core.unicycle_cbf import filter_unicycle_speed, guard_unicycle_step


def test_filter_caps_approach_and_preserves_receding_motion():
    args = dict(
        position=(2, 0),
        heading=np.pi,
        centers=[[0, 0]],
        radii=[1],
        alpha=1,
        max_speed=2,
    )
    approaching = filter_unicycle_speed(nominal_speed=2, **args)
    assert approaching.feasible
    assert approaching.speed == pytest.approx(0.75)
    away = filter_unicycle_speed(nominal_speed=-1, **args)
    assert away.speed == pytest.approx(-1)


def test_infeasible_constraint_reports_no_command():
    result = filter_unicycle_speed((0, 0), 0, 1, [[0, 0]], [1], 1, 2)
    assert not result.feasible
    assert result.speed is None


def test_multiple_obstacles_and_empty_obstacles():
    result = filter_unicycle_speed((0, 0), 0, 1, [[2, 0], [-2, 0]], [1, 1], 1, 2)
    assert result.speed == pytest.approx(0.75)
    result = filter_unicycle_speed((0, 0), 0, 4, [], [], 1, 2)
    assert result.speed == 2


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError):
        filter_unicycle_speed((0, 0), 0, 1, [[0, 0]], [-1], 1, 2)
    with pytest.raises(ValueError):
        filter_unicycle_speed((0, 0), float("nan"), 1, [], [], 1, 2)


def test_stationary_obstacle_approach_remains_outside_in_small_step_pilot():
    """A deterministic pilot checks behavior without claiming general safety."""
    x = 4.0
    for _ in range(100):
        result = filter_unicycle_speed((x, 0), np.pi, 2, [[0, 0]], [1], 1, 2)
        assert result.feasible
        x -= 0.1 * result.speed
        assert x >= 1.0
    assert x == pytest.approx(1.0, abs=0.001)


def test_continuous_constraint_alone_does_not_guarantee_discrete_step_safety():
    result = filter_unicycle_speed((2, 0), np.pi, 2, [[0, 0]], [1], 1, 2)
    assert result.feasible
    # A two-second held command crosses the inflated obstacle boundary.
    assert 2 - 2 * result.speed < 1


def test_step_guard_suppresses_crossing_and_keeps_safe_motion():
    assert guard_unicycle_step((2, 0), np.pi, 0.75, 0, 2, [[0, 0]], [1]) == 0
    assert guard_unicycle_step((2, 0), np.pi, 0.75, 0, 0.1, [[0, 0]], [1]) == 0.75
    assert guard_unicycle_step((2, 0), 0, 1, 0, 1, [[0, 0]], [1]) == 1


def test_step_guard_handles_tangency_reverse_and_start_inside():
    assert guard_unicycle_step((-2, 1), 0, 4, 0, 1, [[0, 0]], [1]) == 0
    assert guard_unicycle_step((2, 0), 0, -2, 0, 1, [[0, 0]], [1]) == 0
    assert guard_unicycle_step((0, 0), 0, 1, 0, 1, [[0, 0]], [1]) is None
