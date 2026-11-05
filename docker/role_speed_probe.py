"""Isolated real ROS agent/gate check for aerial and ground speed limits."""

import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mullet_interfaces.msg import DroneHeartbeat, MissionCommand
import yaml

from docker.deployment_settings import vehicle_endpoint, scenario_path
from docker.ros_agent import SensingAgent


def main():
    rclpy.init()
    with open(scenario_path(), encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    agents = [
        SensingAgent(manifest, v, enable_commands=True) for v in ("uav_1", "uav_3")
    ]
    executor = SingleThreadedExecutor()
    received = {a.vehicle_id: [] for a in agents}
    for agent in agents:
        executor.add_node(agent)
        agent.create_subscription(
            MissionCommand,
            vehicle_endpoint(agent.expected_namespace, "command"),
            lambda msg, v=agent.vehicle_id: received[v].append(msg),
            QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        assert agent.planner.runtime.max_speed == agent.max_speed
    phase = {"speed": 0.8}

    def refresh():
        for agent in agents:
            now = agent.get_clock().now()
            stamp = now.nanoseconds * 1e-9
            hb = DroneHeartbeat()
            hb.drone_id, hb.ros_namespace, hb.schema_version = (
                agent.vehicle_id,
                agent.expected_namespace,
                3,
            )
            hb.stamp, hb.state_label = now.to_msg(), "MISSION"
            hb.armed, hb.local_position_valid, hb.landed = True, True, False
            hb.z = -agent.target_altitude
            agent._heartbeat(hb)
            agent.frame_session = "probe"
            agent.gate.set_offset((0, 0))
            agent.offset_expiry = agent.central_expiry = agent.lease_expiry = (
                stamp + 0.5
            )
            agent.plan = dict(
                scenario_hash=agent.expected_hash,
                vehicle_id=agent.vehicle_id,
                timestamp=stamp,
                expires_at=stamp + 1.0,
                velocity=[phase["speed"], 0.0],
                frame_session="probe",
            )

    timer = agents[0].create_timer(0.04, refresh)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)

    spin_for(2.0)
    assert not received["uav_1"], "ground accepted aerial-speed command"
    assert len(received["uav_3"]) >= 20, "aerial rejected valid faster commands"
    assert all(abs(m.velocity_ned_mps[0] - 0.8) < 1e-6 for m in received["uav_3"])
    phase["speed"] = 1.1
    spin_for(0.3)
    count = len(received["uav_3"])
    spin_for(0.3)
    assert len(received["uav_3"]) == count, "aerial accepted an over-limit command"
    print(
        json.dumps(
            dict(
                passed=True,
                aerial_limit=agents[1].max_speed,
                ground_limit=agents[0].max_speed,
                aerial_commands=count,
                ground_over_limit_rejected=True,
                aerial_over_limit_rejected=True,
            )
        )
    )
    agents[0].destroy_timer(timer)
    for agent in agents:
        executor.remove_node(agent)
        agent.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
