"""Per-vehicle planning with all-to-all pose access within a role team."""

from docker.deployment_ground_runtime import GroundPlannerRuntime
from docker.deployment_planner_runtime import PlannerRuntime


class OnboardPlanner:
    def __init__(self, manifest, scenario_hash, vehicle_id, *, max_speed, max_age):
        roles = {v["id"]: v["role"] for v in manifest["vehicles"]}
        self.vehicle_id = vehicle_id
        self.role = roles[vehicle_id]
        self.team = tuple(v for v, role in roles.items() if role == self.role)
        self.poses = {}
        if self.role == "ground":
            self.runtime = GroundPlannerRuntime(
                scenario_hash, self.team, max_speed=max_speed, max_age=max_age
            )
        elif self.role == "aerial":
            if len(self.team) != 1:
                raise ValueError(
                    "onboard aerial deployment currently requires one aerial vehicle"
                )
            self.runtime = PlannerRuntime(
                scenario_hash, vehicle_id, max_speed=max_speed, max_age=max_age
            )
        else:
            raise ValueError("unsupported planner role")

    def receive_pose(self, packet):
        if packet.get("vehicle_id") in self.team:
            self.poses[packet["vehicle_id"]] = packet

    def plan(self, target, now):
        if self.role == "aerial":
            if self.vehicle_id not in self.poses:
                raise ValueError("missing own pose")
            return self.runtime.plan(target, self.poses[self.vehicle_id], now)
        plans = self.runtime.plan(target, self.poses, now)
        return next(p for p in plans if p["vehicle_id"] == self.vehicle_id)
