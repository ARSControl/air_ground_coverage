"""Deterministic geometry utilities for circular occupancy-grid obstacles."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class CircularObstacleMap:
    """Rasterized, non-overlapping circular obstacles in map coordinates."""

    occupancy: NDArray[np.int32]
    centers: NDArray[np.float64]
    radius: float
    resolution: float


def generate_circular_obstacle_map(
    size: tuple[int, int],
    *,
    count: int,
    radius: float,
    resolution: float,
    seed: int,
    max_attempts_per_obstacle: int = 10_000,
) -> CircularObstacleMap:
    """Generate a seeded binary map containing separated circular obstacles.

    Coordinates are expressed in the same physical units as robot positions.
    A grid cell is occupied when its centre lies inside at least one circle.
    """
    height, width = _validated_size(size)
    if isinstance(count, bool) or not isinstance(count, (int, np.integer)):
        raise TypeError("count must be an integer")
    if count < 0:
        raise ValueError("count must be nonnegative")
    obstacle_radius = _positive_float(radius, "radius")
    cell_resolution = _positive_float(resolution, "resolution")
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
        raise TypeError("seed must be an integer")
    if (
        isinstance(max_attempts_per_obstacle, bool)
        or not isinstance(max_attempts_per_obstacle, (int, np.integer))
        or max_attempts_per_obstacle <= 0
    ):
        raise ValueError("max_attempts_per_obstacle must be a positive integer")

    maximum_x = (width - 1) * cell_resolution
    maximum_y = (height - 1) * cell_resolution
    if 2.0 * obstacle_radius > min(maximum_x, maximum_y):
        raise ValueError("radius is too large for an obstacle to fit inside the map")

    rng = np.random.default_rng(int(seed))
    centers: list[np.ndarray] = []
    for _ in range(int(count)):
        for _attempt in range(int(max_attempts_per_obstacle)):
            candidate = np.array(
                [
                    rng.uniform(obstacle_radius, maximum_x - obstacle_radius),
                    rng.uniform(obstacle_radius, maximum_y - obstacle_radius),
                ],
                dtype=float,
            )
            if all(
                np.linalg.norm(candidate - existing) > 2.0 * obstacle_radius
                for existing in centers
            ):
                centers.append(candidate)
                break
        else:
            raise ValueError(
                "requested obstacles do not fit inside the map without overlap"
            )

    center_array = np.asarray(centers, dtype=float).reshape((-1, 2))
    occupancy = np.zeros((height, width), dtype=np.int32)
    if center_array.size:
        x_coordinates = np.arange(width, dtype=float) * cell_resolution
        y_coordinates = np.arange(height, dtype=float) * cell_resolution
        grid_x, grid_y = np.meshgrid(x_coordinates, y_coordinates)
        squared_radius = obstacle_radius**2
        for center_x, center_y in center_array:
            inside = (grid_x - center_x) ** 2 + (
                grid_y - center_y
            ) ** 2 <= squared_radius
            occupancy[inside] = 1

    occupancy.setflags(write=False)
    center_array.setflags(write=False)
    return CircularObstacleMap(
        occupancy=occupancy,
        centers=center_array,
        radius=obstacle_radius,
        resolution=cell_resolution,
    )


def _validated_size(size: tuple[int, int]) -> tuple[int, int]:
    if not isinstance(size, (tuple, list)) or len(size) != 2:
        raise TypeError("size must contain (height, width)")
    height, width = size
    if any(
        isinstance(value, bool) or not isinstance(value, (int, np.integer))
        for value in (height, width)
    ):
        raise TypeError("size entries must be integers")
    if height < 2 or width < 2:
        raise ValueError("size entries must be at least two")
    return int(height), int(width)


def _positive_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result
