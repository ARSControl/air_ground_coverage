"""Deterministic virtual-clock stress of real planning and command freshness."""

from bisect import bisect_right
from pathlib import Path

import numpy as np
import yaml

from docker.deployment_command_gate import accepted_velocity
from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_publication import CentralUpdateLoop
from docker.deployment_runtime import DeploymentRuntime
from docker.deployment_settings import DeploymentSettings


class VirtualClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def simulate_timing(*, duration=240.0, compute_time=1.45):
    """Hold robot poses fixed; test delivery, planning and gates, not tracking."""
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    model = DeploymentRuntime(manifest)
    identity = model.scenario.scenario_hash
    settings = DeploymentSettings.from_manifest(manifest)
    for vehicle in ("uav_1", "uav_3"):
        model.sample(
            dict(
                scenario_hash=identity,
                vehicle_id=vehicle,
                request_id=vehicle,
                timestamp=0.0,
                position=[15.0, 15.0],
                heading=0.0,
            )
        )
    clock = VirtualClock()
    publications = []
    loop = CentralUpdateLoop(
        model,
        clock=clock,
        wall_clock=clock,
        publish_target=lambda p: publications.append((clock(), p)),
        publish_status=lambda: None,
    )
    loop.tick()
    original = model.tick

    def delayed(timestamp):
        clock.now += compute_time
        return original(timestamp)

    model.tick = delayed
    while clock.now < duration:
        clock.now = loop.last_completed_at + model.estimator.period
        if clock.now > duration:
            break
        loop.tick()
    # The old completion-only pattern had no pre-compute cached publication.
    # Give it ideal completion timestamps; it still exceeds the age limit.
    old = [
        (t, dict(packet, timestamp=t))
        for t, packet in publications
        if not packet["update_in_progress"]
    ]
    times = (
        np.arange(int(round(duration * settings.command_hz)) + 1) / settings.command_hz
    )
    metrics = {}
    ages = {}
    positions = {"uav_1": [10.0, 15.0], "uav_2": [20.0, 15.0], "uav_3": [15.0, 15.0]}
    for label, events in [("old", old), ("fixed", publications)]:
        event_times = [t for t, _ in events]
        planners = {
            v["id"]: OnboardPlanner(
                manifest,
                identity,
                v["id"],
                max_speed=settings.max_speed_for_role(v["role"]),
                max_age=settings.plan_expiry,
            )
            for v in manifest["vehicles"]
        }
        plans = {}
        rejected = checks = 0
        history = []
        for step, timestamp in enumerate(times):
            target = events[bisect_right(event_times, timestamp) - 1][1]
            history.append(timestamp - target["timestamp"])
            if step % int(round(settings.planner_period * settings.command_hz)) == 0:
                packets = {
                    v: dict(
                        scenario_hash=identity,
                        vehicle_id=v,
                        timestamp=timestamp,
                        position=p,
                        heading=0.0,
                        frame_session="timing",
                    )
                    for v, p in positions.items()
                }
                for vehicle, planner in planners.items():
                    for packet in packets.values():
                        planner.receive_pose(packet)
                    try:
                        plans[vehicle] = planner.plan(target, timestamp)
                    except ValueError:
                        plans[vehicle] = None
            for vehicle in manifest["vehicles"]:
                vid = vehicle["id"]
                checks += 1
                command = accepted_velocity(
                    plans.get(vid),
                    identity,
                    vid,
                    timestamp,
                    registered=True,
                    mission=True,
                    max_speed=settings.max_speed_for_role(vehicle["role"]),
                    position=positions[vid],
                    bounds=settings.bounds,
                    margin=settings.margin,
                    horizon=settings.horizon,
                    frame_session="timing",
                    lease_required=True,
                    lease_expiry=timestamp + 0.5,
                )
                rejected += command is None
        ages[label] = np.asarray(history)
        metrics[label + "_command_checks"] = checks
        metrics[label + "_command_rejections"] = rejected
        metrics[label + "_max_target_age_s"] = float(max(history))
    metrics.update(
        duration_s=duration,
        injected_compute_time_s=compute_time,
        pose_and_command_limits_unchanged=True,
        fixed_publications=len(publications),
    )
    return dict(
        times=times, old_age=ages["old"], fixed_age=ages["fixed"], metrics=metrics
    )
