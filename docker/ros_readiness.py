"""Read-only SITL readiness and telemetry recorder. Sends no vehicle commands."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from mullet_interfaces.msg import MissionCommand
import yaml

from docker.deployment_readiness import evaluate_readiness
from docker.deployment_settings import scenario_path
from src.core.deployment_scenario import build_scenario


class ReadinessNode(Node):
    def __init__(self, manifest, event_stream, node_name="mfgp_readiness"):
        super().__init__(node_name)
        self.manifest = manifest
        self.scenario_hash = build_scenario(manifest).scenario_hash
        self.status = self.frames = self.target = None
        self.agents, self.poses, self.plans = {}, {}, {}
        self.commands = {v["id"]: [] for v in manifest["vehicles"]}
        self.events = event_stream
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for topic in (
            "status",
            "frame_offsets",
            "aerial_target",
            "agent_status",
            "pose",
            "proposed_velocity",
            "sample_response",
            "sample_request",
            "mission_lease",
        ):
            self.create_subscription(
                String,
                "/mfgp/" + topic,
                lambda message, name=topic: self._packet(name, message),
                latched
                if topic
                in {"status", "frame_offsets", "aerial_target", "mission_lease"}
                else qos,
            )
        for v in manifest["vehicles"]:
            self.create_subscription(
                MissionCommand,
                v["namespace"] + "/command",
                lambda message, vid=v["id"]: self._command(vid, message),
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

    def _record(self, topic, value):
        self.events.write(
            json.dumps(
                {
                    "topic": topic,
                    "received_at": self.get_clock().now().nanoseconds * 1e-9,
                    "data": value,
                },
                allow_nan=False,
            )
            + "\n"
        )
        self.events.flush()

    def _packet(self, topic, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            self._record(topic, value)
            if topic == "status":
                self.status = value
            elif topic == "aerial_target":
                self.target = value
            elif topic == "frame_offsets":
                self.frames = value
            elif topic in {"agent_status", "pose", "proposed_velocity"} and value.get(
                "vehicle_id"
            ):
                table = {
                    "agent_status": self.agents,
                    "pose": self.poses,
                    "proposed_velocity": self.plans,
                }[topic]
                table[value["vehicle_id"]] = value
        except (ValueError, TypeError):
            return

    def _command(self, vehicle, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.commands[vehicle].append(now)
        self._record(
            "command",
            {
                "vehicle_id": vehicle,
                "mode": int(message.mode),
                "velocity": [float(x) for x in message.velocity_ned_mps[:2]],
                "yaw": float(message.yaw_ned_rad)
                if math.isfinite(message.yaw_ned_rad)
                else None,
            },
        )

    def report(self, require_mission):
        now = self.get_clock().now().nanoseconds * 1e-9
        result = evaluate_readiness(
            self.manifest,
            self.scenario_hash,
            now,
            self.status,
            self.frames,
            self.agents,
            self.poses,
            self.plans,
            require_mission=require_mission,
        )
        stats = {}
        for vehicle, times in self.commands.items():
            gaps = [b - a for a, b in zip(times, times[1:])]
            stats[vehicle] = {
                "received": len(times),
                "last_received_at": times[-1] if times else None,
                "max_gap_s": max(gaps) if gaps else None,
                "mean_rate_hz": (len(times) - 1) / (times[-1] - times[0])
                if len(times) > 1 and times[-1] > times[0]
                else None,
            }
            if require_mission and (not times or now - times[-1] > 0.25):
                result["reasons"].append(f"{vehicle}: no observed fresh ROS command")
        result["command_stats"] = stats
        result["ready"] = not result["reasons"]
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=scenario_path())
    parser.add_argument("--timeout", type=float, default=60)
    parser.add_argument("--in-mission", action="store_true")
    parser.add_argument(
        "--output", type=Path, default=Path("/artifacts/readiness.json")
    )
    parser.add_argument("--record-seconds", type=float, default=0)
    args = parser.parse_args()
    if args.timeout <= 0 or args.record_seconds < 0:
        parser.error("timeout must be positive and recording duration nonnegative")
    with Path(args.scenario).open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    success = False
    with args.output.with_suffix(".events.jsonl").open("w", encoding="utf-8") as events:
        node = ReadinessNode(manifest, events)
        start = last_check = time.monotonic()
        last_progress = start
        stable = 0
        recording_until = None
        try:
            while (
                time.monotonic() - start < args.timeout or recording_until is not None
            ):
                rclpy.spin_once(node, timeout_sec=0.1)
                clock = time.monotonic()
                if clock - last_check < 0.5:
                    continue
                last_check = clock
                result = node.report(args.in_mission)
                if clock - last_progress >= 5.0:
                    node.get_logger().info(
                        "Readiness: "
                        + ("ready" if result["ready"] else "; ".join(result["reasons"]))
                    )
                    last_progress = clock
                stable = stable + 1 if result["ready"] else 0
                if stable >= 3 and recording_until is None:
                    success = True
                    recording_until = clock + args.record_seconds
                if recording_until is not None and clock >= recording_until:
                    break
            result = node.report(args.in_mission)
            result["stable_readiness_observed"] = success
            args.output.write_text(
                json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8"
            )
            print(json.dumps(result, indent=2, allow_nan=False), flush=True)
            success = success and result["ready"]
        finally:
            node.destroy_node()
            rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
