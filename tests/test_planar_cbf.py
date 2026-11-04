import numpy as np
import pytest
from src.core.planar_cbf import filter_velocity


def test_inactive_and_approach():
    assert np.allclose(filter_velocity([0, 0], [0.5, 0], [[10, 0]], [2], 1, 0.5), [0.5, 0])
    v = filter_velocity([2.1, 0], [-0.5, 0.2], [[0, 0]], [2], 1, 0.5)
    assert 4.2 * v[0] >= -0.41 - 1e-10
    assert np.linalg.norm(v) <= 0.5 + 1e-10


def test_multiple_obstacles_bounds_and_interior_hold():
    v = filter_velocity([2.1, 0], [-0.5, 0.2], [[0, 0], [2.1, 3]], [2, 2], 1, 0.5,
                        bounds=[[-5, 5], [-5, 5]], horizon=2.5)
    assert v is not None
    assert filter_velocity([0, 0], [0.5, 0], [[0, 0]], [2], 1, 0.5) is None
    assert filter_velocity([0, 0], [np.nan, 0], [], [], 1, 0.5) is None


def test_deterministic_held_velocity_rollout():
    p = np.array([-6., -1.])
    minimum = np.inf
    for _ in range(600):
        nominal = np.array([6., 1.]) - p
        nominal *= min(1., .5 / np.linalg.norm(nominal))
        v = filter_velocity(p, nominal, [[0, 0], [4, 4]], [2, 2], 1, .5)
        assert v is not None
        segment = .05 * v
        for c in ([0, 0], [4, 4]):
            delta = p - c
            t = np.clip(-delta @ segment / max(segment @ segment, 1e-30), 0, 1)
            minimum = min(minimum, np.linalg.norm(delta + t * segment) - 2)
        p += segment
    assert minimum >= -1e-9
