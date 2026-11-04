"""Pure monotonic deadline state machine for a fixed-duration SITL mission."""

import math


class MissionClock:
    def __init__(self, vehicle_ids, *, duration, service_timeout):
        self.ids = tuple(vehicle_ids)
        if (
            not self.ids
            or len(set(self.ids)) != len(self.ids)
            or any(not isinstance(v, str) or not v for v in self.ids)
            or not all(math.isfinite(x) and x > 0 for x in (duration, service_timeout))
        ):
            raise ValueError("invalid mission team, duration or service timeout")
        self.duration, self.service_timeout = duration, service_timeout
        self.start_results, self.hold_results = {}, {}
        self.state, self.reason = "idle", None
        self.start_time = self.stop_time = None
        self._last_time = None

    def _time(self, now):
        if not math.isfinite(now) or (
            self._last_time is not None and now < self._last_time
        ):
            raise ValueError("mission time must be finite and nondecreasing")
        self._last_time = now

    def begin(self, now):
        self._time(now)
        if self.state != "idle":
            raise ValueError("mission already begun")
        self.start_time = now
        self.state = "starting"
        return "start"

    def tick(self, now, *, cancel=False, healthy=True):
        self._time(now)
        if self.state in {"starting", "running"}:
            reason = None
            if cancel:
                reason = "cancelled"
            elif now >= self.start_time + self.duration:
                reason = "duration_elapsed"
            elif self.state == "starting":
                if any(value is False for value in self.start_results.values()):
                    reason = "start_rejected"
                elif now >= self.start_time + self.service_timeout:
                    reason = "start_timeout"
                elif all(self.start_results.get(vid) is True for vid in self.ids):
                    self.state = "running"
            elif not healthy:
                reason = "pipeline_unhealthy"
            if reason is not None:
                self.state, self.reason, self.stop_time = "stopping", reason, now
                return "hold"
        elif self.state == "stopping":
            if all(vid in self.hold_results for vid in self.ids):
                self.state = (
                    "finished"
                    if all(self.hold_results[vid] for vid in self.ids)
                    else "failed"
                )
                return "finished"
            if now >= self.stop_time + self.service_timeout:
                self.state = "failed"
                return "finished"
        return None

    @property
    def success(self):
        return (
            self.state == "finished"
            and self.reason == "duration_elapsed"
            and all(
                self.start_results.get(v) is True and self.hold_results.get(v) is True
                for v in self.ids
            )
        )


def health_failure_snapshot(report, target=None):
    """Small immutable-by-copy diagnostic captured before requesting HOLD."""
    now = report["timestamp"]

    def age(timestamp):
        if (
            isinstance(timestamp, (int, float))
            and not isinstance(timestamp, bool)
            and math.isfinite(timestamp)
        ):
            return now - timestamp
        return None

    target = target if isinstance(target, dict) else {}
    metadata = {}
    for key in (
        "timestamp",
        "posterior_timestamp",
        "version",
        "update_started_at",
        "update_completed_at",
        "update_duration_s",
    ):
        value = target.get(key)
        metadata[key] = (
            value if isinstance(value, (int, float)) and math.isfinite(value) else None
        )
    vehicles = {}
    for vehicle, packets in report.get("vehicles", {}).items():
        agent = packets.get("agent") or {}
        pose = packets.get("pose") or {}
        plan = packets.get("plan") or {}
        command = report.get("command_stats", {}).get(vehicle, {})
        vehicles[vehicle] = dict(
            state=agent.get("state"),
            pose_age_s=age(pose.get("timestamp")),
            plan_age_s=age(plan.get("timestamp")),
            agent_command_age_s=age(agent.get("last_command_time")),
            observed_command_age_s=age(command.get("last_received_at")),
        )
    central = report.get("central") or {}
    return dict(
        timestamp=now,
        reasons=list(report["reasons"]),
        central_status_age_s=age(central.get("timestamp")),
        target_age_s=age(target.get("timestamp")),
        target=metadata,
        vehicles=vehicles,
    )
