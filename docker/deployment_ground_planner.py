"""Weighted Voronoi coverage proposals for the ground-role drones.

Obstacle avoidance, collision avoidance and command transport are not included.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GroundProposals:
    centroids: np.ndarray
    velocities: np.ndarray
    headings: np.ndarray


def ground_proposals(points, density, weights, positions, headings, *, max_speed, gain):
    """Compute centroid tracking velocities from one synchronized team snapshot.

    The Voronoi assignment and discrete centroids match the existing Lloyd
    implementation. The returned heading-aligned planar velocity adapts its
    unicycle speed formula to a velocity proposal; no robot state is updated.
    """
    points = np.asarray(points, dtype=float)
    density = np.asarray(density, dtype=float)
    weights = np.asarray(weights, dtype=float)
    positions = np.asarray(positions, dtype=float)
    headings = np.asarray(headings, dtype=float)
    if (
        points.ndim != 2
        or points.shape[1:] != (2,)
        or len(points) == 0
        or positions.ndim != 2
        or positions.shape[1:] != (2,)
        or len(positions) == 0
        or density.shape != (len(points),)
        or weights.shape != (len(points),)
        or headings.shape != (len(positions),)
        or not all(
            np.all(np.isfinite(a))
            for a in (points, density, weights, positions, headings)
        )
        or np.any(density < 0)
        or np.any(weights < 0)
        or not np.isfinite(max_speed)
        or max_speed <= 0
        or not np.isfinite(gain)
        or gain <= 0
    ):
        raise ValueError("invalid ground planner inputs")
    assignment = np.argmin(
        np.sum((points[:, None, :] - positions[None, :, :]) ** 2, axis=2), axis=1
    )
    centroids = positions.copy()
    for index in range(len(positions)):
        mass_weights = density * weights * (assignment == index)
        mass = np.sum(mass_weights)
        if mass > np.finfo(float).eps:
            centroids[index] = np.sum(points * mass_weights[:, None], axis=0) / mass
    delta = centroids - positions
    distance = np.linalg.norm(delta, axis=1)
    desired_heading = np.where(
        distance > 0, np.arctan2(delta[:, 1], delta[:, 0]), headings
    )
    error = desired_heading - headings
    speed = np.clip(gain * distance * np.cos(error), -max_speed, max_speed)
    velocities = speed[:, None] * np.column_stack((np.cos(headings), np.sin(headings)))
    return GroundProposals(centroids, velocities, desired_heading)
