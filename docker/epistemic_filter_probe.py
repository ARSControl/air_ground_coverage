"""Network-isolated ROS central filter, HIGH feedback and posterior-refresh probe."""

import json
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_settings import scenario_path
from docker.ros_central import CentralNode
import yaml


def main():
    rclpy.init()
    path = scenario_path()
    with open(path, encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    central = CentralNode(path)
    observer = Node("epistemic_filter_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(central)
    executor.add_node(observer)
    targets = []
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscription = observer.create_subscription(
        String,
        "/mfgp/aerial_target",
        lambda msg: targets.append(json.loads(msg.data)),
        qos,
    )
    publisher = observer.create_publisher(String, "/mfgp/sample_request", 10)
    identity = central.runtime.scenario.scenario_hash

    def spin_until(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "ROS filter probe timeout"

    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)

    def sample(vehicle):
        message = String()
        message.data = json.dumps(
            dict(
                scenario_hash=identity,
                vehicle_id=vehicle,
                request_id=vehicle,
                timestamp=observer.get_clock().now().nanoseconds * 1e-9,
                position=[15.0, 15.0],
                heading=0.0,
            )
        )
        publisher.publish(message)

    sample("uav_3")
    spin_until(lambda: bool(targets))
    before = targets[-1]
    sample("uav_1")
    spin_until(lambda: any(t["version"] > before["version"] for t in targets))
    after = targets[-1]
    assert after["sample_filter"]["accepted_high"] > 0
    assert not np.allclose(before["high_mean"], after["high_mean"])
    assert not np.allclose(before["density"], after["density"])
    velocities = []
    for target in (before, after):
        planner = OnboardPlanner(
            manifest, identity, "uav_3", max_speed=1.0, max_age=2.0
        )
        planner.receive_pose(
            dict(
                scenario_hash=identity,
                vehicle_id="uav_3",
                timestamp=target["timestamp"],
                position=[15.0, 15.0],
                heading=0.0,
                frame_session="probe",
            )
        )
        velocities.append(planner.plan(target, target["timestamp"])["velocity"])
    assert not np.allclose(*velocities)
    spin_until(
        lambda: any(
            t["version"] == after["version"]
            and not t["model_updated"]
            and t["timestamp"] > after["timestamp"]
            for t in targets
        )
    )
    refreshed = targets[-1]
    assert refreshed["posterior_timestamp"] == after["posterior_timestamp"]
    budget = manifest["estimator"]["retention"]["max_total_samples"]
    assert refreshed["sample_filter"]["retained_total"] <= budget
    print(
        json.dumps(
            dict(
                passed=True,
                shared_budget=budget,
                filter=refreshed["sample_filter"],
                posterior_refresh_without_refit=True,
                high_observation_aerial_velocity_change=float(
                    np.linalg.norm(np.diff(velocities, axis=0))
                ),
            )
        )
    )
    observer.destroy_subscription(subscription)
    for node in (central, observer):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
