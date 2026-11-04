"""Transport-independent, in-memory simulated observation service.

This object does not provide ROS, persistence, clock synchronization or flight
readiness. Request IDs must be unique for the lifetime of an instance.
"""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

from .observations import Fidelity, Observation
from ..models.sensors import SimulatedScalarFieldSensor


@dataclass(frozen=True)
class SensorSpec:
    fidelity: Fidelity
    sample_count: int
    range_m: float
    fov_degrees: float
    noise_variance: float
    seed: int
    occlusion: bool


class FieldService:
    """Own immutable field rasters and independent seeded sensor streams."""

    def __init__(
        self,
        scenario_id,
        high_field,
        low_field,
        obstacle_map,
        origin,
        resolution,
        sensors,
        *,
        max_requests=10000,
    ):
        if not isinstance(scenario_id, str) or not scenario_id.strip():
            raise ValueError("scenario ID must be nonempty")
        high = np.asarray(high_field, dtype=float)
        low = np.asarray(low_field, dtype=float)
        raw_map = np.asarray(obstacle_map, dtype=float)
        origin = np.asarray(origin, dtype=float)
        if (
            high.ndim != 2
            or min(high.shape) < 2
            or high.shape != low.shape
            or high.shape != raw_map.shape
            or not np.all(np.isfinite(high))
            or not np.all(np.isfinite(low))
            or not np.all(np.isfinite(raw_map))
            or origin.shape != (2,)
            or not np.all(np.isfinite(origin))
            or not np.isfinite(resolution)
            or resolution <= 0
            or isinstance(max_requests, bool)
            or not isinstance(max_requests, int)
            or max_requests < 1
            or not sensors
        ):
            raise ValueError("invalid field service configuration")
        self._scenario_id = scenario_id
        self._fields = {Fidelity.HIGH: high.copy(), Fidelity.LOW: low.copy()}
        self._origin = origin.copy()
        self._resolution = float(resolution)
        self._sensors = {}
        self._fidelities = {}
        self._cache = {}
        self._max_requests = max_requests
        for vehicle, spec in sensors.items():
            if (
                not isinstance(vehicle, str)
                or not vehicle.strip()
                or not isinstance(spec, SensorSpec)
            ):
                raise ValueError("invalid sensor identity or specification")
            if not isinstance(spec.occlusion, bool):
                raise ValueError("occlusion must be boolean")
            self._sensors[vehicle] = SimulatedScalarFieldSensor(
                spec.fidelity,
                spec.sample_count,
                spec.range_m / self._resolution,
                spec.fov_degrees,
                spec.noise_variance,
                np.random.default_rng(spec.seed),
                obstacle_map=raw_map if spec.occlusion else None,
            )
            self._fidelities[vehicle] = spec.fidelity

    def sample(self, scenario_id, vehicle_id, request_id, timestamp, position, heading):
        """Return a deterministic batch; identical request retries are idempotent."""
        if scenario_id != self._scenario_id:
            raise ValueError("scenario mismatch")
        if vehicle_id not in self._sensors:
            raise ValueError("unknown vehicle")
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("invalid request ID")
        position = np.asarray(position, dtype=float)
        if (
            position.shape != (2,)
            or not np.all(np.isfinite(position))
            or not np.isfinite(timestamp)
            or not np.isfinite(heading)
        ):
            raise ValueError("invalid pose or timestamp")
        grid = (position - self._origin) / self._resolution
        height, width = self._fields[Fidelity.HIGH].shape
        if not (0 <= grid[0] <= width - 1 and 0 <= grid[1] <= height - 1):
            raise ValueError("pose outside map")
        key = (vehicle_id, request_id)
        signature = (
            float(timestamp),
            float(position[0]),
            float(position[1]),
            float(heading),
        )
        if key in self._cache:
            previous, observations = self._cache[key]
            if previous != signature:
                raise ValueError("conflicting duplicate request")
            return observations
        if len(self._cache) >= self._max_requests:
            raise ValueError("request cache full; start a new session")
        team = SimpleNamespace(
            agents=[SimpleNamespace(id=vehicle_id, position=grid, theta=float(heading))]
        )
        fidelity = self._fidelities[vehicle_id]
        raw = self._sensors[vehicle_id].collect(
            team, self._fields[fidelity], float(timestamp)
        )
        observations = tuple(
            Observation(
                o.timestamp,
                o.robot_id,
                tuple(self._origin + self._resolution * np.asarray(o.position)),
                o.value,
                o.fidelity,
                o.noise_variance,
            )
            for o in raw
        )
        self._cache[key] = (signature, observations)
        return observations
