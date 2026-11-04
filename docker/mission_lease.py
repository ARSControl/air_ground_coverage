"""Validate a short-lived timed-mission command-stream lease."""

import math


def lease_expiry(packet, scenario_hash, vehicle_id, now):
    """Return the accepted expiry or zero to revoke permission."""
    try:
        if (
            packet["scenario_hash"] != scenario_hash
            or packet["enabled"] is not True
            or not isinstance(packet["session"], str)
            or not packet["session"]
            or vehicle_id not in packet["vehicle_ids"]
        ):
            return 0.0
        stamp, expiry = float(packet["timestamp"]), float(packet["expires_at"])
        if (
            not all(math.isfinite(x) for x in (stamp, expiry, now))
            or not stamp <= now < expiry
            or not 0 < expiry - stamp <= 0.75
        ):
            return 0.0
        return expiry
    except (KeyError, TypeError, ValueError, OverflowError):
        return 0.0
