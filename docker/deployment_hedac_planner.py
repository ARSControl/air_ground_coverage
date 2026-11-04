"""Experimental aerial HEDAC target-to-velocity adapter; no ROS output."""

from types import SimpleNamespace

import numpy as np

from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm


class AerialHedacPlanner:
    """Apply existing HEDAC heat updates to an externally supplied target."""

    def __init__(
        self, shape, resolution, *, max_speed, origin=(0.0, 0.0), heat_iterations=40
    ):
        if (
            len(shape) != 2
            or min(shape) < 3
            or not np.isfinite(resolution)
            or resolution <= 0
            or not np.isfinite(max_speed)
            or max_speed <= 0
            or not isinstance(heat_iterations, int)
            or heat_iterations < 1
        ):
            raise ValueError("invalid planner configuration")
        self.shape = tuple(shape)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        if self.origin.shape != (2,) or not np.all(np.isfinite(self.origin)):
            raise ValueError("invalid map origin")
        self.max_speed = float(max_speed)
        self.heat_iterations = heat_iterations
        params = HEDACParams.from_dict(
            {
                "simulation": {"dt": 0.2},
                "map": {"resolution": resolution},
                "heat_equation": {
                    "alpha": 1.0,
                    "source_strength": 1.0,
                    "beta": 0.01,
                    "local_cooling": 0.1,
                },
                "agents": {"agent_radius": 1.0, "max_velocity": max_speed},
            }
        )
        loader = MapLoader(size=self.shape, resolution=resolution)
        initial = np.full(self.shape, 1.0 / np.prod(self.shape))
        self._hedac = HEDACAlgorithm(params, loader, initial)

    def step(self, position, heading, target):
        """Update coverage/heat from a measured pose and return map velocity."""
        point = np.asarray(position, dtype=float)
        if (
            point.shape != (2,)
            or not np.all(np.isfinite(point))
            or not np.isfinite(heading)
        ):
            raise ValueError("invalid measured pose")
        grid = (point - self.origin) / self.resolution
        if not (
            0 <= grid[0] <= self.shape[1] - 1 and 0 <= grid[1] <= self.shape[0] - 1
        ):
            raise ValueError("measured pose outside planner grid")
        validated = self._hedac._validate_external_goal_density(target)
        self._hedac.current_goal_density = validated
        agent = SimpleNamespace(position=grid, theta=float(heading))
        self._hedac.update_coverage(agent)
        for _ in range(self.heat_iterations):
            self._hedac.update_heat_field()
        return self.max_speed * self._hedac.get_agent_gradient(agent)
