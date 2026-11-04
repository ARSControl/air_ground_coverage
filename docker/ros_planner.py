"""ROS 2 experimental aerial planner; publishes proposals, not commands."""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from docker.deployment_planner_runtime import PlannerRuntime
from docker.deployment_settings import DeploymentSettings, scenario_path
from src.core.deployment_scenario import build_scenario


class PlannerNode(Node):
    def __init__(self):
        super().__init__("mfgp_aerial_planner")
        self.declare_parameter("scenario", scenario_path())
        self.declare_parameter("vehicle_id", "uav_3")
        with Path(self.get_parameter("scenario").value).open(
            encoding="utf-8"
        ) as stream:
            manifest = yaml.safe_load(stream)
        scenario_hash = build_scenario(manifest).scenario_hash
        settings = DeploymentSettings.from_manifest(manifest)
        vehicle = self.get_parameter("vehicle_id").value
        if not any(
            v["id"] == vehicle and v["role"] == "aerial" for v in manifest["vehicles"]
        ):
            raise ValueError("planner vehicle must be an aerial role")
        self.runtime = PlannerRuntime(
            scenario_hash,
            vehicle,
            max_speed=settings.max_speed_for_role("aerial"),
            max_age=settings.plan_expiry,
        )
        self.target = None
        self.pose = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, "/mfgp/proposed_velocity", qos)
        self.create_subscription(String, "/mfgp/aerial_target", self._target, latched)
        self.create_subscription(String, "/mfgp/pose", self._pose, qos)
        self.create_timer(settings.planner_period, self._plan)
        self.get_logger().info("Proposal-only planner; no flight command publisher")

    def _target(self, message):
        try:
            self.target = json.loads(message.data)
        except (ValueError, TypeError):
            self.target = None

    def _pose(self, message):
        try:
            pose = json.loads(message.data)
            if pose.get("vehicle_id") == self.runtime.vehicle_id:
                self.pose = pose
        except (ValueError, TypeError, AttributeError):
            self.pose = None

    def _plan(self):
        if self.pose is None or self.target is None:
            return
        try:
            plan = self.runtime.plan(
                self.target, self.pose, self.get_clock().now().nanoseconds * 1e-9
            )
            message = String()
            message.data = json.dumps(plan, separators=(",", ":"), allow_nan=False)
            self.publisher.publish(message)
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(
                f"No velocity proposal: {exc}", throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
