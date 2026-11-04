"""Build an in-memory simulated field service from a versioned manifest.

This validates sensing inputs only; it is not a flight-readiness check.
"""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from .deployment_field_service import FieldService, SensorSpec
from .observations import Fidelity
from ..models.sensors import build_fidelity_fields


@dataclass(frozen=True)
class BuiltScenario:
    scenario_hash: str
    obstacle_map: np.ndarray
    service: FieldService
    high_field: np.ndarray


def load_scenario(path: str | Path) -> BuiltScenario:
    """Load YAML and construct a fresh, deterministic field service."""
    with Path(path).open(encoding="utf-8") as stream:
        return build_scenario(yaml.safe_load(stream))


def build_scenario(manifest: dict) -> BuiltScenario:
    """Validate and instantiate the supported static simulation scenario."""
    try:
        return _build(manifest)
    except (KeyError, TypeError, OverflowError, IndexError) as exc:
        raise ValueError("invalid scenario manifest") from exc


def _build(manifest: dict) -> BuiltScenario:
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ValueError("unsupported scenario schema")
    scenario_id = manifest["scenario_id"]
    if not isinstance(scenario_id, str) or not scenario_id.strip():
        raise ValueError("invalid scenario ID")
    try:
        encoded = json.dumps(
            manifest, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    except (ValueError, TypeError) as exc:
        raise ValueError("manifest must contain finite JSON-compatible values") from exc
    digest = hashlib.sha256(encoded).hexdigest()
    grid = manifest["map"]
    if grid["axes"] != "north_east" or grid["row_direction"] != "east":
        raise ValueError("unsupported map axes")
    origin = np.asarray(grid["origin_m"], dtype=float)
    resolution = float(grid["resolution_m"])
    shape = grid["shape_rows_columns"]
    if (
        origin.shape != (2,)
        or not np.all(np.isfinite(origin))
        or not np.isfinite(resolution)
        or resolution <= 0
        or not isinstance(shape, list)
        or len(shape) != 2
        or any(isinstance(v, bool) or not isinstance(v, int) or v < 2 for v in shape)
        or shape[0] * shape[1] > 4_000_000
    ):
        raise ValueError("invalid grid")
    row, col = np.indices(shape, dtype=float)
    x = origin[0] + col * resolution
    y = origin[1] + row * resolution
    obstacles = grid["obstacles"]
    if obstacles["kind"] != "circles":
        raise ValueError("unsupported obstacle geometry")
    occupied = np.zeros(shape, dtype=bool)
    for circle in obstacles["circles"]:
        center = np.asarray(circle["center_m"], dtype=float)
        radius = float(circle["radius_m"])
        if (
            center.shape != (2,)
            or not np.all(np.isfinite(center))
            or not np.isfinite(radius)
            or radius <= 0
        ):
            raise ValueError("invalid obstacle")
        occupied |= (x - center[0]) ** 2 + (y - center[1]) ** 2 <= radius**2
    field = manifest["field"]
    if field["kind"] != "gaussian_mixture" or not field["components"]:
        raise ValueError("unsupported field")
    high = np.zeros(shape, dtype=float)
    for component in field["components"]:
        center = np.asarray(component["center_m"], dtype=float)
        sigma, amplitude = float(component["sigma_m"]), float(component["amplitude"])
        if (
            center.shape != (2,)
            or not np.all(np.isfinite(center))
            or not np.isfinite(sigma)
            or sigma <= 0
            or not np.isfinite(amplitude)
            or amplitude < 0
        ):
            raise ValueError("invalid field component")
        high += amplitude * np.exp(
            -((x - center[0]) ** 2 + (y - center[1]) ** 2) / (2 * sigma**2)
        )
    fields = build_fidelity_fields(
        high, float(field["truth_rho"]), float(field["low_smoothing_sigma_cells"])
    )
    sensors = {}
    for vehicle in manifest["vehicles"]:
        vehicle_id, role, sensor = vehicle["id"], vehicle["role"], vehicle["sensor"]
        if (
            not isinstance(vehicle_id, str)
            or not vehicle_id.strip()
            or vehicle_id in sensors
        ):
            raise ValueError("duplicate or invalid vehicle ID")
        if (
            role == "ground"
            and sensor["fidelity"] == "high"
            and sensor["type"] == "cone"
        ):
            fidelity, radius, fov, occlusion = (
                Fidelity.HIGH,
                sensor["range_m"],
                sensor["fov_degrees"],
                True,
            )
        elif (
            role == "aerial"
            and sensor["fidelity"] == "low"
            and sensor["type"] == "footprint"
            and sensor["occlusion"] is False
        ):
            fidelity, radius, fov, occlusion = (
                Fidelity.LOW,
                sensor["radius_m"],
                360,
                False,
            )
        else:
            raise ValueError("invalid role or sensor")
        sensors[vehicle_id] = SensorSpec(
            fidelity,
            sensor["sample_count"],
            radius,
            fov,
            sensor["noise_variance"],
            sensor["seed"],
            occlusion,
        )
    service = FieldService(
        digest, fields.high, fields.low, occupied, origin, resolution, sensors
    )
    occupied.setflags(write=False)
    return BuiltScenario(digest, occupied, service, fields.high)
