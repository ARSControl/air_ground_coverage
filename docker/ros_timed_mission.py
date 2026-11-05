"""Run a recorded, fixed-duration SITL mission and request HOLD at its end."""

import argparse
import json
import math
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from docker.deployment_settings import vehicle_endpoint, DeploymentSettings, scenario_path
from docker.ros_readiness import ReadinessNode
from docker.timed_mission import MissionClock, health_failure_snapshot


class TimedMissionNode(ReadinessNode):
    def __init__(self, manifest, events):
        super().__init__(manifest, events, node_name="mfgp_timed_mission")
        self.settings = DeploymentSettings.from_manifest(manifest)
        if not self.settings.require_lease:
            raise ValueError("timed missions require require_mission_lease=true")
        self.ids = [v["id"] for v in manifest["vehicles"]]
        self.mission_clients = {
            kind: {
                v["id"]: self.create_client(Trigger, vehicle_endpoint(v["namespace"], suffix))
                for v in manifest["vehicles"]
            }
            for kind, suffix in (("start", "start_mission"), ("hold", "hold"))
        }
        self.lease_publisher = self.create_publisher(
            String,
            "/mfgp/mission_lease",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.lease_session = uuid.uuid4().hex
        self.lease_owned = self.lease_enabled = False
        self.clock = None
        self.stop_ros_time = None
        self.start_ros_time = None
        self.futures = []
        self.create_timer(0.1, self._publish_lease)

    def _publish_lease(self):
        if not self.lease_owned:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        message = String()
        message.data = json.dumps(
            dict(
                scenario_hash=self.scenario_hash,
                session=self.lease_session,
                vehicle_ids=self.ids,
                timestamp=now,
                expires_at=now + 0.5,
                enabled=self.lease_enabled,
            ),
            allow_nan=False,
        )
        self.lease_publisher.publish(message)

    def _call(self, kind, vehicle):
        def done(future):
            try:
                response = future.result()
                ok, detail = bool(response.success), response.message
            except Exception as exc:
                ok, detail = False, str(exc)
            table = (
                self.clock.start_results if kind == "start" else self.clock.hold_results
            )
            table[vehicle] = ok
            self._record(
                "mission_service",
                dict(kind=kind, vehicle_id=vehicle, success=ok, detail=detail),
            )
            if kind == "start" and ok and not self.lease_enabled:
                # A late start acknowledgement after cancellation gets another HOLD.
                self._call("hold", vehicle)

        try:
            future = self.mission_clients[kind][vehicle].call_async(Trigger.Request())
            future.add_done_callback(done)
            self.futures.append(future)
        except Exception as exc:
            table = (
                self.clock.start_results if kind == "start" else self.clock.hold_results
            )
            table[vehicle] = False
            self._record(
                "mission_service",
                dict(kind=kind, vehicle_id=vehicle, success=False, detail=str(exc)),
            )

    def stop_all(self):
        self.lease_enabled = False
        self._publish_lease()
        if self.stop_ros_time is None:
            self.stop_ros_time = self.get_clock().now().nanoseconds * 1e-9
            self._record(
                "mission_lifecycle",
                dict(
                    event="hold_requested",
                    reason=self.clock.reason,
                    timestamp=self.stop_ros_time,
                ),
            )
            for vehicle in self.ids:
                self._call("hold", vehicle)

    def held(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        return {
            vehicle: bool(
                (packet := self.agents.get(vehicle))
                and packet.get("state") == "HOLD"
                and self.stop_ros_time is not None
                and packet["timestamp"] >= self.stop_ros_time
                and 0 <= now - packet["timestamp"] <= self.settings.pose_age
            )
            for vehicle in self.ids
        }


def execute(
    node,
    *,
    duration,
    ready_timeout,
    service_timeout,
    cancelled=lambda: False,
    spin=None,
):
    """Execute through injected spin/clock state, retaining a bounded stop phase."""
    if spin is None:

        def spin(seconds):
            rclpy.spin_once(node, timeout_sec=seconds)

    outcome = dict(success=False, reason="readiness_timeout", duration_s=duration)
    clock = MissionClock(node.ids, duration=duration, service_timeout=service_timeout)
    node.clock = clock
    try:
        wait_until = time.monotonic() + ready_timeout
        stable, last_check = 0, 0.0
        last_progress = time.monotonic()
        while time.monotonic() < wait_until and not cancelled():
            spin(0.05)
            now = time.monotonic()
            if now - last_check < 0.5:
                continue
            last_check = now
            report = node.report(False)
            enabled_hold = all(
                (a := node.agents.get(v))
                and a.get("commands_enabled")
                and a.get("lease_required")
                and a.get("state") == "HOLD"
                for v in node.ids
            )
            services = all(
                client.service_is_ready()
                for table in node.mission_clients.values()
                for client in table.values()
            )
            owner_ok = node.count_publishers("/mfgp/mission_lease") == 1
            ready = report["ready"] and enabled_hold and services and owner_ok
            if now - last_progress >= 5:
                node.get_logger().info(
                    f"Waiting for mission readiness: pipeline={report['ready']}, enabled/HOLD={enabled_hold}, services={services}, sole_runner={owner_ok}"
                )
                last_progress = now
            stable = stable + 1 if ready else 0
            if stable >= 3:
                break
        if stable < 3:
            outcome["reason"] = "cancelled" if cancelled() else "readiness_timeout"
            outcome["readiness_reasons"] = node.report(False)["reasons"] + [
                "All agents must be command-enabled, lease-required and in HOLD; start/hold services and one mission runner must be available"
            ]
            return outcome
        node.lease_owned = node.lease_enabled = True
        clock.begin(time.monotonic())
        node.start_ros_time = node.get_clock().now().nanoseconds * 1e-9
        node._publish_lease()
        node._record(
            "mission_lifecycle",
            dict(
                event="start_requested",
                timestamp=node.start_ros_time,
                duration_s=duration,
            ),
        )
        for vehicle in node.ids:
            node._call("start", vehicle)
        node.get_logger().info(
            f"Mission started; HOLD requested after {duration:g} seconds"
        )
        while clock.state not in {"finished", "failed"}:
            spin(0.05)
            now = time.monotonic()
            health = None if now - clock.start_time < 2.0 else node.report(True)
            healthy = health is None or health["ready"]
            action = clock.tick(now, cancel=cancelled(), healthy=healthy)
            if action == "hold":
                if clock.reason == "pipeline_unhealthy":
                    failure = health_failure_snapshot(health, node.target)
                    outcome["health_failure"] = failure
                    node._record("mission_health_failure", failure)
                    node.get_logger().error(
                        "Mission stopped: " + "; ".join(failure["reasons"])
                    )
                node.stop_all()
        outcome["reason"] = clock.reason
    except Exception as exc:
        outcome["reason"] = "runner_exception"
        outcome["error"] = str(exc)
        if clock.state != "idle":
            clock.reason = "runner_exception"
    finally:
        if clock.state != "idle":
            node.stop_all()
            until = time.monotonic() + service_timeout
            while time.monotonic() < until and not all(node.held().values()):
                spin(0.05)
            outcome.update(
                started_at=node.start_ros_time,
                hold_requested_at=node.stop_ros_time,
                start_results=dict(clock.start_results),
                hold_results=dict(clock.hold_results),
                hold_confirmed=node.held(),
                command_phase_s=(clock.stop_time - clock.start_time)
                if clock.stop_time is not None
                else None,
            )
            outcome["success"] = clock.success and all(node.held().values())
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=scenario_path())
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--ready-timeout", type=float, default=60)
    parser.add_argument("--service-timeout", type=float, default=5)
    parser.add_argument("--output", type=Path, default=Path("/artifacts/mission.json"))
    args = parser.parse_args()
    if not all(
        math.isfinite(v) and v > 0
        for v in (args.ready_timeout, args.service_timeout, args.duration)
    ):
        parser.error("timeouts must be positive")
    # Validate the duration before creating any ROS publishers or clients.
    MissionClock(
        ["validation"], duration=args.duration, service_timeout=args.service_timeout
    )
    with Path(args.scenario).open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    stopped = {"value": False}
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: stopped.update(value=True))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    with args.output.with_suffix(".events.jsonl").open("w", encoding="utf-8") as events:
        node = TimedMissionNode(manifest, events)
        try:
            outcome = execute(
                node,
                duration=args.duration,
                ready_timeout=args.ready_timeout,
                service_timeout=args.service_timeout,
                cancelled=lambda: stopped["value"],
            )
            report = node.report(False)
            report["mission"] = outcome
            report["stable_readiness_observed"] = node.start_ros_time is not None
            args.output.write_text(
                json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
            print(json.dumps(outcome, indent=2, allow_nan=False), flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    raise SystemExit(0 if outcome["success"] else 1)


if __name__ == "__main__":
    main()
