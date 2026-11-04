"""Isolated ROS wiring check; no command publishers or MULLET services used."""

import json
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import yaml

from docker.deployment_settings import scenario_path
from docker.ros_agent import SensingAgent


def main():
    rclpy.init()
    with open(scenario_path(), encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    agents = [SensingAgent(manifest, v["id"]) for v in manifest["vehicles"]]
    executor = SingleThreadedExecutor()
    for agent in agents:
        executor.add_node(agent)
    y, x = np.indices((21, 21))
    density = np.exp(-((x - 15) ** 2 + (y - 10) ** 2) / 4).ravel().tolist()
    observed = []
    agents[0].create_subscription(
        String,
        "/mfgp/proposed_velocity",
        lambda msg: observed.append(json.loads(msg.data)),
        10,
    )

    def refresh():
        now = agents[0].get_clock().now().nanoseconds * 1e-9
        target = dict(
            scenario_hash=agents[0].expected_hash,
            timestamp=now,
            version=1,
            shape=[21, 21],
            query_points=np.column_stack((x.ravel(), y.ravel())).tolist(),
            density=density,
            high_density=density,
            integration_weights=np.ones(441).tolist(),
        )
        for i, agent in enumerate(agents):
            message = String()
            message.data = json.dumps(target)
            agent._target(message)
            pose = dict(
                scenario_hash=agent.expected_hash,
                vehicle_id=agent.vehicle_id,
                timestamp=now,
                position=[5.0 + 5 * i, 10.0],
                heading=0.0,
                frame_session="probe",
            )
            agent.planner.receive_pose(pose)
            message.data = json.dumps(pose)
            agent.team_pose_publisher.publish(message)

    timer = agents[0].create_timer(0.1, refresh)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert {p["vehicle_id"] for p in observed} == {a.vehicle_id for a in agents}
    assert set(agents[0].planner.poses) == {"uav_1", "uav_2"}
    assert set(agents[2].planner.poses) == {"uav_3"}
    assert all(a.command_publisher is None for a in agents)
    assert all(a.plan["vehicle_id"] == a.vehicle_id for a in agents)
    # An external proposal cannot replace any agent's locally computed command input.
    assert not hasattr(agents[0], "_proposal")
    agents[0].destroy_timer(timer)
    agents[0].planner.poses.pop("uav_2")
    agents[0]._plan()
    assert agents[0].plan is None
    print(
        json.dumps(
            dict(
                passed=True,
                vehicles=3,
                proposals=len(observed),
                role_isolation=True,
                missing_peer_clears_plan=True,
            )
        )
    )
    for agent in agents:
        executor.remove_node(agent)
        agent.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
