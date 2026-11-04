"""Isolated ROS posterior/target replay of a full-budget improving swap."""

import json
import tempfile
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_settings import scenario_path
from docker.ros_central import CentralNode
from src.core.observations import Fidelity, Observation


def main():
    with open(scenario_path(), encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    manifest["estimator"]["retention"]["max_total_samples"] = 3
    rclpy.init()
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
        yaml.safe_dump(manifest, stream)
        stream.flush()
        central = CentralNode(stream.name)
    observer = Node("replacement_probe")
    executor = SingleThreadedExecutor()
    for node in (central, observer):
        executor.add_node(node)
    targets = []
    subscription = observer.create_subscription(
        String,
        "/mfgp/aerial_target",
        lambda m: targets.append(json.loads(m.data)),
        QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    identity = central.runtime.scenario.scenario_hash
    now = observer.get_clock().now().nanoseconds * 1e-9
    for vehicle, batch in [
        (
            "uav_3",
            (
                Observation(now, "uav_3", (15.0, 15.0), 0.2, Fidelity.LOW, 0.04),
                Observation(now, "uav_3", (15.05, 15.0), 0.2, Fidelity.LOW, 0.04),
            ),
        ),
        (
            "uav_1",
            (Observation(now, "uav_1", (40.0, 40.0), 0.2, Fidelity.HIGH, 0.0001),),
        ),
    ]:
        central.runtime.estimator.submit_batch(identity, vehicle, "initial", batch)

    def spin_until(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "replacement ROS probe timed out"

    spin_until(lambda: bool(targets))
    before = targets[-1]
    now = observer.get_clock().now().nanoseconds * 1e-9
    central.runtime.estimator.submit_batch(
        identity,
        "uav_1",
        "swap",
        (Observation(now, "uav_1", (15.0, 15.0), 1.0, Fidelity.HIGH, 0.0001),),
    )
    spin_until(lambda: any(t["sample_filter"]["replaced"] == 1 for t in targets))
    after = targets[-1]
    diagnostics = after["sample_filter"]
    assert diagnostics["retained_total"] == 3 and diagnostics["evicted_low"] == 1
    assert (
        diagnostics["last_replacement_objective_after"]
        < diagnostics["last_replacement_objective_before"]
    )
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
                position=[20.0, 15.0],
                heading=0.0,
                frame_session="probe",
            )
        )
        velocities.append(planner.plan(target, target["timestamp"])["velocity"])
    assert not np.allclose(*velocities)
    spin_until(
        lambda: any(
            not t["model_updated"] and t["timestamp"] > after["timestamp"]
            for t in targets
        )
    )
    print(
        json.dumps(
            dict(
                passed=True,
                filter=diagnostics,
                aerial_velocity_change=float(
                    np.linalg.norm(np.diff(velocities, axis=0))
                ),
                posterior_refresh_without_refit=True,
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
