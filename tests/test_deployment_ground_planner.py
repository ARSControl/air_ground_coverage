"""Ground-role weighted Voronoi proposal contracts."""

import numpy as np
import pytest

from docker.deployment_ground_planner import ground_proposals
from src.coupled_simulation import compute_weighted_voronoi_centroids
from src.utils.voronoi import compute_voronoi_partitioning


def test_centroids_match_existing_coverage_implementation():
    points = np.array([[0, 0], [1, 0], [2, 0], [3, 0]], dtype=float)
    positions = np.array([[0, 0], [3, 0]], dtype=float)
    density = np.array([1, 2, 3, 4], dtype=float)
    weights = np.array([0.5, 1, 1, 0.5])
    expected, _ = compute_weighted_voronoi_centroids(
        points,
        density,
        np.asarray(compute_voronoi_partitioning(points, positions, 100)),
        weights,
        positions,
    )
    result = ground_proposals(
        points, density, weights, positions, [0, 0], max_speed=1, gain=1
    )
    np.testing.assert_allclose(result.centroids, expected)
    assert np.all(np.linalg.norm(result.velocities, axis=1) <= 1 + 1e-12)


def test_empty_mass_keeps_position_and_invalid_data_rejected():
    result = ground_proposals([[0, 0]], [0], [1], [[0, 0]], [0], max_speed=1, gain=1)
    np.testing.assert_allclose(result.velocities, [[0, 0]])
    with pytest.raises(ValueError):
        ground_proposals([[0, 0]], [-1], [1], [[0, 0]], [0], max_speed=1, gain=1)
