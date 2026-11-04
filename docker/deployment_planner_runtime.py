"""Validate ROS planning inputs and produce proposed aerial velocities."""

import math

import numpy as np

from docker.deployment_hedac_planner import AerialHedacPlanner


class PlannerRuntime:
    """One HEDAC controller for one registered aerial-role vehicle."""

    def __init__(self, scenario_hash, vehicle_id, *, max_speed, max_age):
        if (
            not scenario_hash
            or not vehicle_id
            or not math.isfinite(max_age)
            or max_age <= 0
        ):
            raise ValueError("invalid planner identity or age")
        if not math.isfinite(max_speed) or max_speed <= 0:
            raise ValueError("invalid speed")
        self.scenario_hash = scenario_hash
        self.vehicle_id = vehicle_id
        self.max_speed = max_speed
        self.max_age = max_age
        self._planner = None
        self._grid_signature = None

    def plan(self, target, pose, now):
        """Return a short-lived proposal; never publish flight commands."""
        if (
            target["scenario_hash"] != self.scenario_hash
            or pose["scenario_hash"] != self.scenario_hash
            or pose["vehicle_id"] != self.vehicle_id
        ):
            raise ValueError("scenario or vehicle mismatch")
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("invalid clock")
        for timestamp in (target["timestamp"], pose["timestamp"]):
            if not math.isfinite(timestamp) or not 0 <= now - timestamp <= self.max_age:
                raise ValueError("stale or future target or pose")
        shape = tuple(target["shape"])
        if len(shape) != 2 or any(
            isinstance(n, bool) or not isinstance(n, int) or n < 3 or n > 201
            for n in shape
        ):
            raise ValueError("invalid grid shape")
        points = np.asarray(target["query_points"], dtype=float)
        density = np.asarray(target["density"], dtype=float)
        if (
            points.shape != (shape[0] * shape[1], 2)
            or density.shape != (len(points),)
            or not np.all(np.isfinite(points))
            or not np.all(np.isfinite(density))
            or np.any(density < 0)
        ):
            raise ValueError("invalid target arrays")
        points = points.reshape((*shape, 2))
        xs, ys = points[0, :, 0], points[:, 0, 1]
        dx, dy = float(xs[1] - xs[0]), float(ys[1] - ys[0])
        xx, yy = np.meshgrid(xs, ys)
        if (
            dx <= 0
            or dy <= 0
            or not np.isclose(dx, dy)
            or not np.allclose(np.diff(xs), dx)
            or not np.allclose(np.diff(ys), dy)
            or not np.allclose(points[:, :, 0], xx)
            or not np.allclose(points[:, :, 1], yy)
        ):
            raise ValueError("target grid must be square-spaced and structured")
        signature = (shape, float(xs[0]), float(ys[0]), dx)
        if self._planner is not None and signature != self._grid_signature:
            raise ValueError("target grid changed")
        if self._planner is None:
            self._planner = AerialHedacPlanner(
                shape, dx, max_speed=self.max_speed, origin=(xs[0], ys[0])
            )
            self._grid_signature = signature
        velocity = self._planner.step(
            pose["position"], pose["heading"], density.reshape(shape)
        )
        return {
            "scenario_hash": self.scenario_hash,
            "vehicle_id": self.vehicle_id,
            "posterior_version": target["version"],
            "timestamp": now,
            "expires_at": now + self.max_age,
            "velocity": velocity.tolist(),
            "frame_session": pose.get("frame_session"),
        }
