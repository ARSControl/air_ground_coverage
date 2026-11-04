"""ROS 2 wrapper for the experimental central sensing and MFGP runtime.

This process publishes no vehicle commands and is not a flight controller.
"""

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from docker.deployment_runtime import DeploymentRuntime
from docker.deployment_publication import CentralUpdateLoop
from docker.deployment_settings import DeploymentSettings, scenario_path


class CentralNode(Node):
    def __init__(self, scenario_path):
        super().__init__("mfgp_central")
        with Path(scenario_path).open(encoding="utf-8") as stream:
            manifest = yaml.safe_load(stream)
            self.settings = DeploymentSettings.from_manifest(manifest)
            self.runtime = DeploymentRuntime(manifest)
        self.error_count = 0
        self.last_error = ""
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(String, "/mfgp/status", latched)
        self.response_publisher = self.create_publisher(
            String, "/mfgp/sample_response", qos
        )
        self.target_publisher = self.create_publisher(
            String, "/mfgp/aerial_target", latched
        )
        self.updates = CentralUpdateLoop(
            self.runtime,
            clock=lambda: self.get_clock().now().nanoseconds * 1e-9,
            publish_target=lambda value: self.target_publisher.publish(
                self._message(value)
            ),
            publish_status=self._status,
        )
        self.create_subscription(String, "/mfgp/sample_request", self._on_request, qos)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._status)
        self._status()

    @staticmethod
    def _message(value):
        msg = String()
        msg.data = json.dumps(value, separators=(",", ":"), allow_nan=False)
        return msg

    def _status(self):
        self.status_publisher.publish(
            self._message(
                {
                    "scenario_hash": self.runtime.scenario.scenario_hash,
                    "state": "sensing_only",
                    "flight_ready": False,
                    "timestamp": self.get_clock().now().nanoseconds * 1e-9,
                    "posterior_version": self.runtime.estimator.estimator.version,
                    "low_sample_count": self.runtime.estimator.estimator.low_sample_count,
                    "high_sample_count": self.runtime.estimator.estimator.high_sample_count,
                    "error_count": self.error_count,
                    "last_error": self.last_error,
                    "sample_filter": self.runtime.estimator.estimator.filter_diagnostics,
                    "update_timing": self.updates.diagnostics,
                }
            )
        )

    def _on_request(self, message):
        try:
            request = json.loads(message.data)
            now = self.get_clock().now().nanoseconds * 1e-9
            age = now - float(request["timestamp"])
            if not 0 <= age <= self.settings.observation_age:
                raise ValueError("sample request is stale or future")
            response = {"ok": True, **self.runtime.sample(request)}
        except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
            response = {"ok": False, "error": str(exc)}
            self.error_count += 1
            self.last_error = str(exc)
        self.response_publisher.publish(self._message(response))

    def _tick(self):
        try:
            self.updates.tick()
        except (ValueError, RuntimeError) as exc:
            self.error_count += 1
            self.last_error = str(exc)
            self.get_logger().error(f"central update failed: {exc}")
            self._status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=scenario_path())
    args = parser.parse_args()
    rclpy.init()
    node = CentralNode(args.scenario)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
