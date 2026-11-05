"""Explicit deployment settings shared by the ROS processes."""

from dataclasses import dataclass
import math
import os
import re


def valid_vehicle_namespace(namespace):
    return isinstance(namespace, str) and (
        namespace == '/'
        or re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace)
        is not None
    )


def vehicle_endpoint(namespace, suffix):
    """Resolve a relative MULLET endpoint under a validated ROS namespace."""
    if not valid_vehicle_namespace(namespace) or not isinstance(suffix, str) or re.fullmatch(
        r'[A-Za-z_][A-Za-z0-9_]*', suffix
    ) is None:
        raise ValueError('invalid vehicle namespace or endpoint')
    return namespace.rstrip('/') + '/' + suffix


def scenario_path():
    return os.environ.get(
        "MFGP_SCENARIO", "/opt/mfgp/docker/scenarios/sitl_three_drone.yaml"
    )


@dataclass(frozen=True)
class DeploymentSettings:
    bounds: tuple
    command_hz: float
    planner_period: float
    max_speed: float
    plan_expiry: float
    altitude_tolerance: float
    margin: float
    horizon: float
    pose_age: float
    observation_age: float
    require_lease: bool = True
    role_max_speeds: tuple = ()

    def max_speed_for_role(self, role):
        if role not in {"aerial", "ground"}:
            raise ValueError("unsupported vehicle role")
        return dict(self.role_max_speeds).get(role, self.max_speed)

    @classmethod
    def from_manifest(cls, manifest):
        try:
            control, timing, grid = (
                manifest["control"],
                manifest["time"],
                manifest["map"],
            )
            if timing["clock_domain"] != "system":
                raise ValueError("current deployment requires a common system clock")
            keys = (
                "command_stream_hz",
                "planner_period_s",
                "max_speed_mps",
                "plan_expiry_s",
                "altitude_tolerance_m",
                "geofence_margin_m",
                "stopping_horizon_s",
            )
            values = [float(control[k]) for k in keys]
            values += [
                float(timing["max_pose_age_s"]),
                float(timing["max_observation_age_s"]),
            ]
            vehicles = manifest["vehicles"]
            ids, namespaces = set(), set()
            for vehicle in vehicles:
                vid, namespace = vehicle["id"], vehicle["namespace"]
                altitude = float(vehicle["altitude_m"])
                if (
                    not isinstance(vid, str)
                    or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", vid) is None
                    or vid in ids
                    or not valid_vehicle_namespace(namespace)
                    or namespace in namespaces
                    or not math.isfinite(altitude)
                    or altitude <= 0
                ):
                    raise ValueError(
                        "invalid or duplicate vehicle identity, namespace or altitude"
                    )
                ids.add(vid)
                namespaces.add(namespace)
            if not all(math.isfinite(v) and v > 0 for v in values) or values[3] > 2.0:
                raise ValueError("invalid deployment rates or limits")
            north, east = (float(v) for v in grid["origin_m"])
            rows, cols = grid["shape_rows_columns"]
            resolution = float(grid["resolution_m"])
            bounds = (
                (north, north + (cols - 1) * resolution),
                (east, east + (rows - 1) * resolution),
            )
            if not all(math.isfinite(v) for b in bounds for v in b) or any(
                b[1] - b[0] <= 2 * values[5] for b in bounds
            ):
                raise ValueError("invalid deployment bounds")
            require_lease = control.get("require_mission_lease", True)
            if not isinstance(require_lease, bool):
                raise ValueError("require_mission_lease must be boolean")
            role_limits = control.get("max_speed_by_role_mps", {})
            if not isinstance(role_limits, dict) or set(role_limits) - {
                "aerial",
                "ground",
            }:
                raise ValueError("invalid role speed mapping")
            parsed_limits = []
            for role, raw in role_limits.items():
                if isinstance(raw, bool):
                    raise ValueError("role speeds must be positive finite numbers")
                limit = float(raw)
                if not math.isfinite(limit) or limit <= 0:
                    raise ValueError("role speeds must be positive finite numbers")
                parsed_limits.append((role, limit))
            return cls(bounds, *values, require_lease, tuple(parsed_limits))
        except (KeyError, TypeError, OverflowError) as exc:
            raise ValueError("incomplete deployment settings") from exc
