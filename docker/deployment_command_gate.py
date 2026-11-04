"""Fail-closed acceptance of bounded, expiring planar velocity proposals."""

import math


def accepted_velocity(
    plan,
    scenario_hash,
    vehicle_id,
    now,
    *,
    registered,
    mission,
    max_speed=0.5,
    position=None,
    bounds=None,
    margin=0.0,
    horizon=2.5,
    frame_session=None,
    lease_required=False,
    lease_expiry=None,
    obstacle_filter=None,
):
    """Return north/east velocity and optional yaw, or None on rejection.

    Registration and mission booleans must come from fresh independently
    validated state. This is not obstacle avoidance or a flight safety proof.
    """
    if not registered or not mission:
        return None
    if lease_required and (
        not isinstance(lease_expiry, (float, int))
        or not math.isfinite(lease_expiry)
        or not now < lease_expiry
    ):
        return None
    try:
        if plan["scenario_hash"] != scenario_hash or plan["vehicle_id"] != vehicle_id:
            return None
        if frame_session is not None and plan.get("frame_session") != frame_session:
            return None
        timestamp, expiry = float(plan["timestamp"]), float(plan["expires_at"])
        north, east = (float(v) for v in plan["velocity"])
        if not all(
            math.isfinite(v) for v in (now, timestamp, expiry, north, east, max_speed)
        ):
            return None
        if max_speed <= 0 or not timestamp <= now < expiry or expiry - timestamp > 2.0:
            return None
        if math.hypot(north, east) > max_speed + 1e-9:
            return None
        if obstacle_filter is not None and obstacle_filter.enabled:
            if position is None:
                return None
            safe_bounds = None if bounds is None else [(lo + margin, hi - margin) for lo, hi in bounds]
            north, east = obstacle_filter.apply(position, (north, east), max_speed, safe_bounds, horizon)
        if bounds is not None:
            if (
                position is None
                or len(position) != 2
                or len(bounds) != 2
                or not math.isfinite(margin)
                or margin < 0
                or not math.isfinite(horizon)
                or horizon <= 0
            ):
                return None
            for coordinate, velocity, interval in zip(position, (north, east), bounds):
                low, high = interval
                if (
                    not all(math.isfinite(v) for v in (coordinate, low, high))
                    or not low + margin <= coordinate <= high - margin
                    or not low + margin
                    <= coordinate + horizon * velocity
                    <= high - margin
                ):
                    return None
        heading = plan.get("heading")
        if heading is not None:
            heading = float(heading)
            if not math.isfinite(heading):
                return None
        return north, east, heading
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
