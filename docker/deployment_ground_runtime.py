"""Freshness-gated ground-role velocity proposals; never sends commands."""

import math

from docker.deployment_ground_planner import ground_proposals


class GroundPlannerRuntime:
    def __init__(self, scenario_hash, vehicle_ids, *, max_speed, max_age):
        ids = tuple(vehicle_ids)
        if (
            not scenario_hash
            or not ids
            or len(set(ids)) != len(ids)
            or any(not isinstance(v, str) or not v for v in ids)
        ):
            raise ValueError("invalid planner identity")
        if (
            not math.isfinite(max_speed)
            or max_speed <= 0
            or not math.isfinite(max_age)
            or max_age <= 0
        ):
            raise ValueError("invalid speed or freshness")
        self.scenario_hash, self.vehicle_ids = scenario_hash, ids
        self.max_speed, self.max_age = max_speed, max_age

    def plan(self, target, poses, now):
        """Require all configured fresh poses and one matching HIGH posterior."""
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("invalid planner clock")
        if target["scenario_hash"] != self.scenario_hash:
            raise ValueError("target scenario mismatch")
        missing = set(self.vehicle_ids) - set(poses)
        extra = set(poses) - set(self.vehicle_ids)
        if missing:
            raise ValueError("waiting for missing team poses: " + ", ".join(sorted(missing)))
        if extra:
            raise ValueError("unexpected team poses: " + ", ".join(sorted(extra)))
        packets = [poses[vehicle_id] for vehicle_id in self.vehicle_ids]
        if len({p.get("frame_session") for p in packets}) != 1:
            raise ValueError("ground poses have different registration sessions")
        for vehicle_id, packet in zip(self.vehicle_ids, packets):
            if (
                packet["vehicle_id"] != vehicle_id
                or packet["scenario_hash"] != self.scenario_hash
            ):
                raise ValueError("pose identity mismatch")
        for timestamp in [target["timestamp"], *(p["timestamp"] for p in packets)]:
            if not math.isfinite(timestamp) or not 0 <= now - timestamp <= self.max_age:
                raise ValueError("stale or future input")
        result = ground_proposals(
            target["query_points"],
            target["high_density"],
            target["integration_weights"],
            [p["position"] for p in packets],
            [p["heading"] for p in packets],
            max_speed=self.max_speed,
            gain=1.0,
        )
        return [
            {
                "scenario_hash": self.scenario_hash,
                "vehicle_id": vehicle_id,
                "posterior_version": target["version"],
                "timestamp": now,
                "expires_at": now + self.max_age,
                "velocity": result.velocities[i].tolist(),
                "heading": float(result.headings[i]),
                "frame_session": packets[i].get("frame_session"),
            }
            for i, vehicle_id in enumerate(self.vehicle_ids)
        ]
