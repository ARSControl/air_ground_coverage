"""Isolated ROS command serialization probe. Run with Docker network none."""

import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mullet_interfaces.msg import DroneHeartbeat, MissionCommand
import yaml
from docker.deployment_settings import scenario_path

from docker.ros_agent import SensingAgent


def main():
    rclpy.init()
    with open(scenario_path(), encoding="utf-8") as stream:
        node = SensingAgent(yaml.safe_load(stream), "uav_1", enable_commands=True)
    received = []
    subscription = node.create_subscription(
        MissionCommand,
        "/uav_1/command",
        received.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    phase = {"name": "valid"}

    def refresh():
        now = node.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        heartbeat = DroneHeartbeat()
        heartbeat.drone_id = "uav_1"
        heartbeat.ros_namespace = (
            "/wrong_vehicle"
            if phase["name"] == "namespace"
            else node.expected_namespace
        )
        heartbeat.schema_version = 3
        heartbeat.stamp = now.to_msg()
        heartbeat.state_label = "HOLD" if phase["name"] == "hold" else "MISSION"
        heartbeat.armed = True
        heartbeat.local_position_valid = True
        heartbeat.landed = False
        heartbeat.z = -node.target_altitude
        if phase["name"] == "altitude":
            heartbeat.z -= 2
        if phase["name"] == "stale_pose":
            heartbeat.stamp.sec -= 2
        node._heartbeat(heartbeat)
        node.frame_session = "probe"
        node.gate.set_offset((0, 0))
        node.offset_expiry = stamp - 1 if phase["name"] == "frame" else stamp + 0.5
        node.central_expiry = stamp + 0.5
        node.lease_expiry = stamp - 1 if phase["name"] == "lease" else stamp + 0.5
        node.plan = dict(
            scenario_hash=node.expected_hash,
            vehicle_id="uav_1",
            timestamp=stamp - 2 if phase["name"] == "plan" else stamp,
            expires_at=stamp - 1 if phase["name"] == "plan" else stamp + 1,
            velocity=[0.3, 0.4],
            heading=0.2,
            frame_session="probe",
        )

    def spin_for(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)

    timer = node.create_timer(0.04, refresh)
    spin_for(2.0)
    assert len(received) >= 20, "no sustained command stream in isolated ROS graph"
    command = received[-1]
    assert command.mode == MissionCommand.VELOCITY
    assert abs(command.velocity_ned_mps[0] - 0.3) < 1e-6
    assert abs(command.velocity_ned_mps[1] - 0.4) < 1e-6
    assert math.isnan(command.velocity_ned_mps[2])
    assert all(math.isnan(v) for v in command.position_ned_m)
    assert all(math.isnan(v) for v in command.acceleration_ned_mps2)
    stamps = [m.stamp.sec + m.stamp.nanosec * 1e-9 for m in received]
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    assert 15 <= rate <= 25, f"unexpected command rate {rate}"
    assert max(gaps) < 0.25, "command gap exceeds MULLET freshness window"
    for name in (
        "plan",
        "frame",
        "stale_pose",
        "altitude",
        "hold",
        "namespace",
        "lease",
    ):
        phase["name"] = name
        spin_for(0.25)
        count = len(received)
        spin_for(0.35)
        assert len(received) == count, f"commands continue after {name} invalidation"
    import json

    print(
        json.dumps(
            {
                "passed": True,
                "command_rate_hz": rate,
                "max_command_gap_s": max(gaps),
                "rejection_cases": 7,
            }
        ),
        flush=True,
    )
    node.destroy_timer(timer)
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
