"""ROS ground-role coverage proposals; never publishes flight commands."""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from docker.deployment_ground_runtime import GroundPlannerRuntime
from docker.deployment_settings import DeploymentSettings, scenario_path
from src.core.deployment_scenario import build_scenario


class GroundPlannerNode(Node):
    def __init__(self):
        super().__init__("mfgp_ground_planner")
        self.declare_parameter("scenario", scenario_path())
        with Path(self.get_parameter("scenario").value).open(
            encoding="utf-8"
        ) as stream:
            manifest = yaml.safe_load(stream)
        scenario_hash = build_scenario(manifest).scenario_hash
        settings = DeploymentSettings.from_manifest(manifest)
        ids = [v["id"] for v in manifest["vehicles"] if v["role"] == "ground"]
        self.runtime = GroundPlannerRuntime(
            scenario_hash,
            ids,
            max_speed=settings.max_speed_for_role("ground"),
            max_age=settings.plan_expiry,
        )
        self.target = None
        self.poses = {}
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
        self.get_logger().info(
            "Proposal-only ground planner; no obstacle avoidance or flight commands"
        )

    def _target(self, message):
        try:
            self.target = json.loads(message.data)
        except (ValueError, TypeError):
            self.target = None

    def _pose(self, message):
        try:
            pose = json.loads(message.data)
            vehicle = pose.get("vehicle_id")
            if vehicle in self.runtime.vehicle_ids:
                self.poses[vehicle] = pose
        except (ValueError, TypeError, AttributeError):
            return

    def _plan(self):
        if self.target is None or set(self.poses) != set(self.runtime.vehicle_ids):
            return
        try:
            plans = self.runtime.plan(
                self.target, self.poses, self.get_clock().now().nanoseconds * 1e-9
            )
            for plan in plans:
                message = String()
                message.data = json.dumps(plan, separators=(",", ":"), allow_nan=False)
                self.publisher.publish(message)
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(
                f"No ground proposals: {exc}", throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    node = GroundPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
