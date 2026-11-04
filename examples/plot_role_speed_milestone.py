"""Compare real onboard proposals before/after configuring role speed limits."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_runtime import DeploymentRuntime
from docker.deployment_settings import DeploymentSettings


def main():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    runtime = DeploymentRuntime(manifest)
    identity = runtime.scenario.scenario_hash
    for vehicle in ("uav_1", "uav_3"):
        runtime.sample(
            dict(
                scenario_hash=identity,
                vehicle_id=vehicle,
                request_id="visual",
                timestamp=0.0,
                position=[15.0, 15.0],
                heading=0.0,
            )
        )
    target = runtime.tick(0.0)
    poses = {
        vehicle: dict(
            scenario_hash=identity,
            vehicle_id=vehicle,
            timestamp=0.0,
            position=p,
            heading=0.0,
            frame_session="visual",
        )
        for vehicle, p in [
            ("uav_1", [10.0, 15.0]),
            ("uav_2", [20.0, 15.0]),
            ("uav_3", [15.0, 15.0]),
        ]
    }
    settings = DeploymentSettings.from_manifest(manifest)
    results = {}
    for label in ("Shared speed", "Role speeds"):
        results[label] = {}
        for vehicle in manifest["vehicles"]:
            limit = (
                settings.max_speed
                if label == "Shared speed"
                else settings.max_speed_for_role(vehicle["role"])
            )
            planner = OnboardPlanner(
                manifest, identity, vehicle["id"], max_speed=limit, max_age=2.0
            )
            for pose in poses.values():
                planner.receive_pose(pose)
            plan = planner.plan(target, 0.1)
            results[label][vehicle["id"]] = dict(
                limit_mps=limit,
                velocity_mps=plan["velocity"],
                speed_mps=float(np.linalg.norm(plan["velocity"])),
            )
    fig, ax = plt.subplots(figsize=(7, 4), layout="constrained")
    ids = list(poses)
    x = np.arange(len(ids))
    width = 0.35
    for offset, label, color in [
        (-width / 2, "Shared speed", "#a3a3a3"),
        (width / 2, "Role speeds", "#2563eb"),
    ]:
        ax.bar(
            x + offset,
            [results[label][v]["speed_mps"] for v in ids],
            width,
            label=label,
            color=color,
        )
    ax.set(
        xticks=x,
        xticklabels=["uav_1 (ground)", "uav_2 (ground)", "uav_3 (aerial)"],
        ylabel="Onboard proposed speed [m/s]",
        title="Same posterior and measured poses; changed role limits",
        ylim=(0, 1.15),
    )
    ax.legend()
    root = Path("docs/assets/sitl")
    fig.savefig(root / "role_speeds.png", dpi=150)
    plt.close(fig)
    (root / "role_speeds_metrics.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
