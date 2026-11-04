"""Read-only pose-to-sampling-request checks for a deployment agent."""

import math


def _point(value):
    try:
        x, y = value
        result = (float(x), float(y))
    except (TypeError, ValueError) as exc:
        raise ValueError("expected two finite coordinates") from exc
    if not all(math.isfinite(v) for v in result):
        raise ValueError("expected two finite coordinates")
    return result


class SamplingGate:
    """Create sample requests from valid fresh poses with a known NED offset.

    Callers must validate reference alignment, offset quality and reset state.
    This class does not establish those properties or provide flight safety.
    """

    def __init__(self, vehicle_id, session_id, offset, *, max_pose_age):
        if not all(isinstance(v, str) and v.strip() for v in (vehicle_id, session_id)):
            raise ValueError("vehicle and session IDs must be nonempty")
        if not math.isfinite(max_pose_age) or max_pose_age <= 0:
            raise ValueError("invalid maximum pose age")
        self.vehicle_id = vehicle_id
        self.session_id = session_id
        self.max_pose_age = float(max_pose_age)
        self.offset = None
        self.scenario = None
        self.sequence = 0
        if offset is not None:
            self.set_offset(offset)

    def set_offset(self, offset):
        self.offset = _point(offset)

    def clear_offset(self):
        self.offset = None

    def set_scenario(self, scenario_hash):
        if not isinstance(scenario_hash, str) or not scenario_hash:
            raise ValueError("invalid scenario hash")
        self.scenario = scenario_hash

    def request(self, now, pose_time, position, heading, *, valid):
        if not valid or self.offset is None or self.scenario is None:
            return None
        try:
            now, pose_time, heading = float(now), float(pose_time), float(heading)
            x, y = _point(position)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (now, pose_time, heading)):
            return None
        if not 0 <= now - pose_time <= self.max_pose_age:
            return None
        request = {
            "scenario_hash": self.scenario,
            "vehicle_id": self.vehicle_id,
            "request_id": f"{self.session_id}:{self.sequence}",
            "timestamp": pose_time,
            "position": [x + self.offset[0], y + self.offset[1]],
            "heading": heading,
        }
        self.sequence += 1
        return request
