"""Publish tentative map offsets from synchronized MULLET heartbeats.

This is a sensing-only SITL prototype, not validated flight localization.
"""

import json
from pathlib import Path
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from mullet_interfaces.msg import DroneHeartbeat
import yaml

from docker.deployment_frame_registry import FrameRegistry, FrameSample
from docker.deployment_settings import scenario_path
from src.core.deployment_scenario import build_scenario


class FrameNode(Node):
    def __init__(self):
        super().__init__("mfgp_frame_registry")
        self.declare_parameter("scenario", scenario_path())
        with Path(self.get_parameter("scenario").value).open(
            encoding="utf-8"
        ) as stream:
            manifest = yaml.safe_load(stream)
        self.scenario_hash = build_scenario(manifest).scenario_hash
        self.registry = FrameRegistry.from_manifest(manifest)
        self.session = uuid.uuid4().hex
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, "/mfgp/frame_offsets", latched)
        self.create_subscription(
            DroneHeartbeat, "/mullet/heartbeat", self._on_heartbeat, qos
        )
        self.create_timer(0.25, self._publish)
        self.get_logger().info(
            "SITL frame registry; estimates are not flight validated"
        )

    def _on_heartbeat(self, heartbeat):
        vehicle = heartbeat.drone_id
        if vehicle not in self.registry.ids:
            return
        if not (
            heartbeat.local_position_valid
            and heartbeat.global_position_valid
            and heartbeat.armed
            and not heartbeat.landed
        ):
            self.registry.invalidate(vehicle)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        timestamp = heartbeat.stamp.sec + heartbeat.stamp.nanosec * 1e-9
        try:
            self.registry.observe(
                vehicle,
                FrameSample(
                    timestamp,
                    (heartbeat.x, heartbeat.y),
                    (heartbeat.lat, heartbeat.lon),
                    int(heartbeat.sequence),
                ),
                now,
            )
        except ValueError:
            self.registry.invalidate(vehicle)

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        message = String()
        message.data = json.dumps(
            {
                "scenario_hash": self.scenario_hash,
                "session": f"{self.session}:{self.registry.generation}",
                "timestamp": now,
                "expires_at": now + 0.75,
                "offsets": self.registry.offsets(now),
                "flight_validated": False,
                "reference_fault": self.registry.reference_fault,
            },
            separators=(",", ":"),
            allow_nan=False,
        )
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = FrameNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
