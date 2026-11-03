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


def line_of_sight_is_clear(
    occupancy: NDArray[np.generic],
    start: tuple[float, float] | NDArray[np.floating],
    end: tuple[float, float] | NDArray[np.floating],
) -> bool:
    """Return whether a closed 2-D segment avoids occupied raster cells.

    Raster cell ``(y, x)`` represents the closed unit square centred at
    ``(x, y)``. Contact with an occupied cell boundary is conservatively
    classified as occluded. Both segment endpoints must lie inside the map's
    centre-coordinate bounds.
    """
    raw_occupancy = np.asarray(occupancy)
    if raw_occupancy.ndim != 2:
        raise ValueError("occupancy must be a two-dimensional raster")
    if raw_occupancy.shape[0] < 1 or raw_occupancy.shape[1] < 1:
        raise ValueError("occupancy must be nonempty")
    try:
        finite_occupancy = np.asarray(raw_occupancy, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("occupancy must be numeric or boolean") from exc
    if not np.all(np.isfinite(finite_occupancy)):
        raise ValueError("occupancy must contain only finite values")
    blocked = finite_occupancy != 0.0
    segment_start = _finite_point(start, "start")
    segment_end = _finite_point(end, "end")
    height, width = blocked.shape
    for point, name in ((segment_start, "start"), (segment_end, "end")):
        if not (0.0 <= point[0] <= width - 1 and 0.0 <= point[1] <= height - 1):
            raise ValueError(f"{name} must lie inside the map bounds")

    if not np.any(blocked):
        return True

    minimum = np.minimum(segment_start, segment_end)
    maximum = np.maximum(segment_start, segment_end)
    x_min = max(0, int(math.ceil(minimum[0] - 0.5)))
    x_max = min(width - 1, int(math.floor(maximum[0] + 0.5)))
    y_min = max(0, int(math.ceil(minimum[1] - 0.5)))
    y_max = min(height - 1, int(math.floor(maximum[1] + 0.5)))
    occupied_cells = np.argwhere(blocked[y_min : y_max + 1, x_min : x_max + 1])
    for local_y, local_x in occupied_cells:
        cell_x = x_min + int(local_x)
        cell_y = y_min + int(local_y)
        if _segment_intersects_closed_cell(
            segment_start,
            segment_end,
            cell_x,
            cell_y,
        ):
            return False
    return True


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


def _finite_point(
    value: tuple[float, float] | NDArray[np.floating], name: str
) -> NDArray[np.float64]:
    try:
        point = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite two-value coordinate") from exc
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError(f"{name} must be a finite two-value coordinate")
    return np.array(point, dtype=float, copy=True)


def _segment_intersects_closed_cell(
    start: NDArray[np.float64],
    end: NDArray[np.float64],
    cell_x: int,
    cell_y: int,
) -> bool:
    """Apply a slab intersection test to one closed unit raster cell."""
    displacement = end - start
    lower = np.array((cell_x - 0.5, cell_y - 0.5), dtype=float)
    upper = np.array((cell_x + 0.5, cell_y + 0.5), dtype=float)
    entry = 0.0
    exit_ = 1.0
    tolerance = 1.0e-12
    for axis in range(2):
        if abs(displacement[axis]) <= tolerance:
            if (
                start[axis] < lower[axis] - tolerance
                or start[axis] > upper[axis] + tolerance
            ):
                return False
            continue
        first = (lower[axis] - start[axis]) / displacement[axis]
        second = (upper[axis] - start[axis]) / displacement[axis]
        axis_entry = min(first, second)
        axis_exit = max(first, second)
        entry = max(entry, axis_entry)
        exit_ = min(exit_, axis_exit)
        if entry > exit_ + tolerance:
            return False
    return entry <= 1.0 + tolerance and exit_ >= -tolerance


def _positive_float(value: float, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result
