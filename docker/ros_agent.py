"""MULLET sensing agent with explicitly enabled planar command streaming."""

import argparse
import json
from pathlib import Path
import uuid
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from mullet_interfaces.msg import DroneHeartbeat, MissionCommand
import yaml

from docker.deployment_agent_gate import SamplingGate
from docker.deployment_command_gate import accepted_velocity
from docker.deployment_obstacle_filter import ObstacleFilter
from docker.deployment_settings import DeploymentSettings, scenario_path
from docker.mission_lease import lease_expiry
from docker.deployment_onboard_planner import OnboardPlanner
from src.core.deployment_scenario import build_scenario


class SensingAgent(Node):
    def __init__(self, manifest, vehicle_id, enable_commands=False):
        super().__init__(f"mfgp_sensing_{vehicle_id}")
        scenarios = [
            vehicle for vehicle in manifest["vehicles"] if vehicle["id"] == vehicle_id
        ]
        if len(scenarios) != 1:
            raise ValueError("vehicle ID not uniquely present in scenario")
        self.vehicle_id = vehicle_id
        self.expected_namespace = scenarios[0]["namespace"]
        self.settings = DeploymentSettings.from_manifest(manifest)
        self.max_speed = self.settings.max_speed_for_role(scenarios[0]["role"])
        self.obstacle_filter = ObstacleFilter(manifest, scenarios[0]["role"])
        self.target_altitude = float(scenarios[0]["altitude_m"])
        self.commands_enabled = enable_commands
        self.commands_sent = 0
        self.last_command_time = None
        self.central_expiry = 0.0
        self.lease_expiry = 0.0
        self.expected_hash = build_scenario(manifest).scenario_hash
        self.gate = SamplingGate(
            vehicle_id, uuid.uuid4().hex, None, max_pose_age=self.settings.pose_age
        )
        self.offset_expiry = 0.0
        self.frame_session = None
        self.heartbeat = None
        self.plan = None
        self.target = None
        self.planner = OnboardPlanner(
            manifest,
            self.expected_hash,
            vehicle_id,
            max_speed=self.max_speed,
            max_age=self.settings.plan_expiry,
        )
        self.command_publisher = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.proposal_publisher = self.create_publisher(
            String, "/mfgp/proposed_velocity", qos
        )
        self.create_subscription(String, "/mfgp/aerial_target", self._target, latched)
        self.create_subscription(
            String, f"/mfgp/team_pose/{self.planner.role}", self._peer_pose, qos
        )
        self.team_pose_publisher = self.create_publisher(
            String, f"/mfgp/team_pose/{self.planner.role}", qos
        )
        self.create_timer(self.settings.planner_period, self._plan)
        self.publisher = self.create_publisher(String, "/mfgp/sample_request", qos)
        self.pose_publisher = self.create_publisher(String, "/mfgp/pose", qos)
        self.agent_status_publisher = self.create_publisher(
            String, "/mfgp/agent_status", qos
        )
        self.create_timer(0.5, self._agent_status)
        self.create_subscription(
            DroneHeartbeat, "/mullet/heartbeat", self._heartbeat, qos
        )
        self.create_subscription(String, "/mfgp/status", self._status, latched)
        self.create_subscription(String, "/mfgp/frame_offsets", self._offsets, latched)
        self.create_subscription(String, "/mfgp/mission_lease", self._lease, latched)
        if enable_commands:
            self.command_publisher = self.create_publisher(
                MissionCommand,
                scenarios[0]["namespace"] + "/command",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_timer(1.0 / self.settings.command_hz, self._command)
        self.create_timer(float(scenarios[0]["sensor"]["period_s"]), self._sample)
        self.get_logger().info(
            f"Agent command output enabled: {enable_commands}; MULLET must enter MISSION externally"
        )

    def _target(self, message):
        try:
            self.target = json.loads(message.data)
        except (ValueError, TypeError):
            self.target = None

    def _peer_pose(self, message):
        try:
            self.planner.receive_pose(json.loads(message.data))
        except (ValueError, TypeError, AttributeError):
            return

    def _plan(self):
        if self.target is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            plan = self.planner.plan(self.target, now)
            message = String()
            message.data = json.dumps(plan, separators=(",", ":"), allow_nan=False)
            self.plan = plan
            self.proposal_publisher.publish(message)
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.plan = None
            self.get_logger().warning(
                f"No onboard proposal: {exc}", throttle_duration_sec=5.0
            )

    def _lease(self, message):
        try:
            self.lease_expiry = lease_expiry(
                json.loads(message.data),
                self.expected_hash,
                self.vehicle_id,
                self.get_clock().now().nanoseconds * 1e-9,
            )
        except (ValueError, TypeError):
            self.lease_expiry = 0.0

    def _command(self):
        heartbeat = self.heartbeat
        if self.command_publisher is None or heartbeat is None or self.plan is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        pose_time = heartbeat.stamp.sec + heartbeat.stamp.nanosec * 1e-9
        fresh = 0 <= now - pose_time <= self.settings.pose_age
        offset = self.gate.offset
        position = (
            None
            if offset is None
            else (heartbeat.x + offset[0], heartbeat.y + offset[1])
        )
        velocity = accepted_velocity(
            self.plan,
            self.expected_hash,
            self.vehicle_id,
            now,
            registered=offset is not None
            and now < self.offset_expiry
            and now < self.central_expiry,
            mission=fresh
            and heartbeat.local_position_valid
            and heartbeat.armed
            and not heartbeat.landed
            and not heartbeat.px4_failsafe
            and self._altitude_ok()
            and heartbeat.state_label == "MISSION",
            max_speed=self.max_speed,
            position=position,
            bounds=self.settings.bounds,
            margin=self.settings.margin,
            horizon=self.settings.horizon,
            frame_session=self.frame_session,
            lease_required=self.settings.require_lease,
            lease_expiry=self.lease_expiry,
            obstacle_filter=self.obstacle_filter,
        )
        if velocity is None:
            return
        if self.obstacle_filter.last_status == "unsafe_or_infeasible_hold":
            self.get_logger().warning("Obstacle CBF unsafe/infeasible: commanding hold", throttle_duration_sec=5.0)
        north, east, heading = velocity
        command = MissionCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.mode = MissionCommand.VELOCITY
        command.position_ned_m = [math.nan] * 3
        command.velocity_ned_mps = [north, east, math.nan]
        command.acceleration_ned_mps2 = [math.nan] * 3
        command.yaw_ned_rad = math.nan if heading is None else heading
        self.command_publisher.publish(command)
        self.commands_sent += 1
        self.last_command_time = now

    def _altitude_ok(self):
        return (
            self.heartbeat is not None
            and math.isfinite(self.heartbeat.z)
            and abs(-self.heartbeat.z - self.target_altitude)
            <= self.settings.altitude_tolerance
        )

    def _agent_status(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        hb = self.heartbeat
        pose_valid = (
            hb is not None
            and hb.local_position_valid
            and hb.armed
            and not hb.landed
            and not hb.px4_failsafe
            and 0
            <= now - (hb.stamp.sec + hb.stamp.nanosec * 1e-9)
            <= self.settings.pose_age
        )
        value = dict(
            scenario_hash=self.expected_hash,
            vehicle_id=self.vehicle_id,
            timestamp=now,
            registered=self.gate.offset is not None and now < self.offset_expiry,
            pose_valid=bool(pose_valid),
            altitude_ok=bool(self._altitude_ok()),
            state=hb.state_label if hb is not None else "UNKNOWN",
            commands_enabled=self.commands_enabled,
            max_speed_mps=self.max_speed,
            commands_sent=self.commands_sent,
            obstacle_cbf_status=self.obstacle_filter.last_status,
            last_command_time=self.last_command_time,
            frame_session=self.frame_session,
            lease_required=self.settings.require_lease,
            lease_valid=now < self.lease_expiry,
        )
        message = String()
        message.data = json.dumps(value, allow_nan=False)
        self.agent_status_publisher.publish(message)

    def _heartbeat(self, message):
        if message.drone_id == self.vehicle_id:
            self.heartbeat = (
                message
                if message.ros_namespace == self.expected_namespace
                and message.schema_version == 3
                else None
            )

    def _status(self, message):
        try:
            value = json.loads(message.data)
            if (
                value.get("scenario_hash") == self.expected_hash
                and value.get("state") == "sensing_only"
            ):
                self.gate.set_scenario(self.expected_hash)
                self.central_expiry = self.get_clock().now().nanoseconds * 1e-9 + 2.0
            else:
                self.gate.scenario = None
                self.central_expiry = 0.0
        except (ValueError, TypeError, AttributeError):
            self.gate.scenario = None
            self.central_expiry = 0.0

    def _offsets(self, message):
        self.gate.clear_offset()
        self.offset_expiry = 0.0
        self.frame_session = None
        try:
            value = json.loads(message.data)
            now = self.get_clock().now().nanoseconds * 1e-9
            timestamp, expiry = float(value["timestamp"]), float(value["expires_at"])
            if (
                value["scenario_hash"] != self.expected_hash
                or not timestamp <= now < expiry
                or expiry - timestamp > 1.0
                or not isinstance(value.get("session"), str)
                or not value["session"]
                or value.get("reference_fault", False)
            ):
                return
            offset = value["offsets"].get(self.vehicle_id)
            if offset is not None:
                self.gate.set_offset(offset)
                self.offset_expiry = expiry
                self.frame_session = value["session"]
        except (ValueError, TypeError, KeyError, AttributeError):
            return

    def _sample(self):
        heartbeat = self.heartbeat
        if heartbeat is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now >= self.offset_expiry or now >= self.central_expiry:
            self.gate.clear_offset()
            return
        timestamp = heartbeat.stamp.sec + heartbeat.stamp.nanosec * 1e-9
        request = self.gate.request(
            now,
            timestamp,
            (heartbeat.x, heartbeat.y),
            heartbeat.yaw,
            valid=heartbeat.local_position_valid
            and heartbeat.armed
            and not heartbeat.landed
            and heartbeat.state_label in {"HOLD", "MISSION"},
        )
        if request is not None:
            pose = {
                key: request[key]
                for key in (
                    "scenario_hash",
                    "vehicle_id",
                    "timestamp",
                    "position",
                    "heading",
                )
            }
            pose_message = String()
            pose["frame_session"] = self.frame_session
            pose_message.data = json.dumps(pose, separators=(",", ":"), allow_nan=False)
            self.planner.receive_pose(pose)
            self.pose_publisher.publish(pose_message)
            self.team_pose_publisher.publish(pose_message)
            message = String()
            message.data = json.dumps(request, separators=(",", ":"), allow_nan=False)
            self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=scenario_path())
    parser.add_argument("--vehicle", required=True)
    parser.add_argument(
        "--enable-commands",
        action="store_true",
        default=os.environ.get("ENABLE_COMMANDS", "0") == "1",
    )
    args = parser.parse_args()
    with Path(args.scenario).open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    rclpy.init()
    node = SensingAgent(manifest, args.vehicle, args.enable_commands)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
