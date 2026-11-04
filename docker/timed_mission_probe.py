"""Network-isolated integration test with emulated MULLET Trigger services."""

import io
import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mullet_interfaces.msg import MissionCommand
import yaml

from docker.mission_lease import lease_expiry
from docker.ros_timed_mission import TimedMissionNode, execute
from docker.deployment_settings import scenario_path
from src.core.deployment_scenario import build_scenario


class FakeMullet(Node):
    def __init__(self, manifest, reject=None, enabled=True, unhealthy=False):
        super().__init__("fake_mullet_timed_probe")
        self.ids = [v["id"] for v in manifest["vehicles"]]
        self.scenario_hash = build_scenario(manifest).scenario_hash
        self.states = {v: "HOLD" for v in self.ids}
        self.start_calls, self.hold_calls = (
            {v: 0 for v in self.ids},
            {v: 0 for v in self.ids},
        )
        self.counts, self.last_command = (
            {v: 0 for v in self.ids},
            {v: None for v in self.ids},
        )
        self.lease = None
        self.reject = reject
        self.enabled = enabled
        self.unhealthy = unhealthy
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publishers_by_topic = {
            topic: self.create_publisher(
                String,
                "/mfgp/" + topic,
                latched if topic in {"status", "frame_offsets"} else qos,
            )
            for topic in (
                "status",
                "frame_offsets",
                "agent_status",
                "pose",
                "proposed_velocity",
            )
        }
        self.command_pubs = {
            v["id"]: self.create_publisher(
                MissionCommand,
                v["namespace"] + "/command",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            for v in manifest["vehicles"]
        }
        for v in manifest["vehicles"]:
            for kind in ("start_mission", "hold"):
                self.create_service(
                    Trigger,
                    v["namespace"] + "/" + kind,
                    lambda request, response, vid=v["id"], action=kind: self._service(
                        vid, action, response
                    ),
                )
        self.create_subscription(String, "/mfgp/mission_lease", self._lease, latched)
        self.create_timer(0.1, self._telemetry)
        self.create_timer(0.05, self._commands)

    def _lease(self, message):
        self.lease = json.loads(message.data)

    def _service(self, vehicle, kind, response):
        if kind == "start_mission":
            self.start_calls[vehicle] += 1
            response.success = vehicle != self.reject and self.states[vehicle] == "HOLD"
            if response.success:
                self.states[vehicle] = "MISSION"
        else:
            self.hold_calls[vehicle] += 1
            response.success = self.states[vehicle] == "MISSION"
            self.states[vehicle] = "HOLD"
        response.message = "fake service for isolated test"
        return response

    def _publish(self, topic, value):
        msg = String()
        msg.data = json.dumps(value, allow_nan=False)
        self.publishers_by_topic[topic].publish(msg)

    def _commands(self):
        now = self.get_clock().now()
        seconds = now.nanoseconds * 1e-9
        for vid in self.ids:
            if (
                self.states[vid] == "MISSION"
                and lease_expiry(self.lease, self.scenario_hash, vid, seconds) > seconds
            ):
                msg = MissionCommand()
                msg.stamp = now.to_msg()
                msg.mode = MissionCommand.VELOCITY
                msg.velocity_ned_mps = [0.1, 0.0, float("nan")]
                self.command_pubs[vid].publish(msg)
                self.counts[vid] += 1
                self.last_command[vid] = seconds

    def _telemetry(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        base = dict(scenario_hash=self.scenario_hash, timestamp=now)
        self._publish(
            "status",
            {
                **base,
                "posterior_version": 0
                if self.unhealthy and any(v == "MISSION" for v in self.states.values())
                else 1,
                "low_sample_count": 3,
                "high_sample_count": 6,
            },
        )
        self._publish(
            "frame_offsets",
            {
                **base,
                "session": "fake",
                "expires_at": now + 0.75,
                "offsets": {v: [0, 0] for v in self.ids},
            },
        )
        for vid in self.ids:
            specific = {**base, "vehicle_id": vid, "frame_session": "fake"}
            self._publish("pose", {**specific, "position": [0, 0], "heading": 0})
            self._publish(
                "proposed_velocity",
                {
                    **specific,
                    "velocity": [0.1, 0],
                    "posterior_version": 1,
                    "expires_at": now + 2,
                },
            )
            self._publish(
                "agent_status",
                {
                    **specific,
                    "registered": True,
                    "pose_valid": True,
                    "altitude_ok": True,
                    "commands_enabled": self.enabled,
                    "lease_required": True,
                    "lease_valid": lease_expiry(
                        self.lease, self.scenario_hash, vid, now
                    )
                    > now,
                    "state": self.states[vid],
                    "commands_sent": self.counts[vid],
                    "last_command_time": self.last_command[vid],
                },
            )


def check(manifest, reject=None, cancel=False, enabled=True, unhealthy=False):
    fake = FakeMullet(manifest, reject=reject, enabled=enabled, unhealthy=unhealthy)
    stream = io.StringIO()
    runner = TimedMissionNode(manifest, stream)
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(runner)
    try:
        result = execute(
            runner,
            duration=10.0 if cancel or unhealthy else 1.0,
            ready_timeout=10 if enabled else 2,
            service_timeout=3,
            spin=lambda seconds: executor.spin_once(timeout_sec=seconds),
            cancelled=lambda: bool(
                cancel
                and runner.clock is not None
                and runner.clock.start_time is not None
                and time.monotonic() - runner.clock.start_time >= 0.2
            ),
        )
        if not enabled:
            assert not any(fake.start_calls.values()) and not any(
                fake.hold_calls.values()
            ), result
            assert not result["success"] and result["reason"] == "readiness_timeout", (
                result
            )
            return result
        assert all(count >= 1 for count in fake.start_calls.values()), result
        assert all(count >= 1 for count in fake.hold_calls.values()), result
        assert all(state == "HOLD" for state in fake.states.values()), result
        if cancel:
            assert not result["success"] and result["reason"] == "cancelled", result
        elif unhealthy:
            assert result["reason"] == "pipeline_unhealthy" and not result["success"], (
                result
            )
            assert result["health_failure"]["reasons"], result
            assert any(
                "valid posterior" in reason
                for reason in result["health_failure"]["reasons"]
            ), result
        elif reject is None:
            assert result["success"], result
            assert 1 <= result["command_phase_s"] <= 1.3, result
        else:
            assert not result["success"] and result["reason"] == "start_rejected", (
                result
            )
        return result
    finally:
        executor.shutdown()
        runner.destroy_node()
        fake.destroy_node()


def main():
    with open(scenario_path(), encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    rclpy.init()
    try:
        normal = check(manifest)
        rejected = check(manifest, reject="uav_2")
        cancelled = check(manifest, cancel=True)
        disabled = check(manifest, enabled=False)
        unhealthy = check(manifest, unhealthy=True)
        print(
            json.dumps(
                {
                    "passed": True,
                    "normal": normal,
                    "partial_start": rejected,
                    "cancelled": cancelled,
                    "disabled_agents": disabled,
                    "pipeline_unhealthy": unhealthy,
                }
            ),
            flush=True,
        )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
