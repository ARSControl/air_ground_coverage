"""Continuous-time circular-obstacle CBF speed filter for a unicycle.

This mathematical prototype is not a discrete-time collision guarantee or a
flight safety filter. The caller supplies inflated obstacle radii, and must
handle infeasibility, map boundaries, tracking errors and sample-and-hold.
"""

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike


@dataclass(frozen=True)
class SpeedFilterResult:
    feasible: bool
    speed: float | None


def filter_unicycle_speed(
    position: ArrayLike,
    heading: float,
    nominal_speed: float,
    centers: ArrayLike,
    radii: ArrayLike,
    alpha: float,
    max_speed: float,
) -> SpeedFilterResult:
    """Project nominal speed onto all circular CBF and speed constraints."""
    position = np.asarray(position, dtype=float)
    centers = np.asarray(centers, dtype=float)
    radii = np.asarray(radii, dtype=float)
    values = np.asarray([heading, nominal_speed, alpha, max_speed], dtype=float)
    if centers.size == 0:
        centers = centers.reshape((0, 2))
    if (
        position.shape != (2,)
        or centers.ndim != 2
        or centers.shape[1:] != (2,)
        or radii.shape != (len(centers),)
        or not np.all(np.isfinite(position))
        or not np.all(np.isfinite(centers))
        or not np.all(np.isfinite(radii))
        or not np.all(np.isfinite(values))
        or np.any(radii <= 0)
        or alpha <= 0
        or max_speed < 0
    ):
        raise ValueError("invalid CBF geometry or parameters")
    direction = np.array([math.cos(heading), math.sin(heading)])
    lower, upper = -max_speed, max_speed
    for center, radius in zip(centers, radii):
        delta = position - center
        coefficient = float(2 * np.dot(delta, direction))
        rhs = float(-alpha * (np.dot(delta, delta) - radius * radius))
        if not math.isfinite(coefficient) or not math.isfinite(rhs):
            raise ValueError("CBF arithmetic overflow")
        if abs(coefficient) <= 1e-12:
            if rhs > 0:
                return SpeedFilterResult(False, None)
        elif coefficient > 0:
            lower = max(lower, rhs / coefficient)
        else:
            upper = min(upper, rhs / coefficient)
    if lower > upper:
        return SpeedFilterResult(False, None)
    return SpeedFilterResult(True, float(np.clip(nominal_speed, lower, upper)))


def guard_unicycle_step(
    position: ArrayLike,
    heading: float,
    speed: float,
    angular_speed: float,
    dt: float,
    centers: ArrayLike,
    radii: ArrayLike,
) -> float | None:
    """Suppress translation if the simulation's next straight segment touches a disk.

    Return None if already inside or on an inflated disk. This matches the
    unicycle integrator's heading-first step, not a physical drone trajectory.
    """
    if not np.all(np.isfinite([heading, speed, angular_speed, dt])) or dt <= 0:
        raise ValueError("invalid step parameters")
    filter_unicycle_speed(position, heading, speed, centers, radii, 1.0, abs(speed))
    point = np.asarray(position, dtype=float)
    obstacle_centers = np.asarray(centers, dtype=float).reshape((-1, 2))
    obstacle_radii = np.asarray(radii, dtype=float)
    next_heading = heading + angular_speed * dt
    segment = speed * dt * np.array([math.cos(next_heading), math.sin(next_heading)])
    length_squared = float(np.dot(segment, segment))
    if not math.isfinite(length_squared):
        raise ValueError("step arithmetic overflow")
    for center, radius in zip(obstacle_centers, obstacle_radii):
        delta = point - center
        if np.dot(delta, delta) <= radius * radius:
            return None
        fraction = (
            float(np.clip(-np.dot(delta, segment) / length_squared, 0.0, 1.0))
            if length_squared > 0
            else 0.0
        )
        closest = delta + fraction * segment
        if np.dot(closest, closest) <= radius * radius:
            return 0.0
    return float(speed)
