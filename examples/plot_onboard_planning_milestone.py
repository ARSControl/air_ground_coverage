"""Deterministic real-estimator output and per-drone planner validation summary.

Run from repository root: PYTHONPATH=. .venv/bin/python examples/plot_onboard_planning_milestone.py
"""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from docker.deployment_ground_runtime import GroundPlannerRuntime
from docker.deployment_onboard_planner import OnboardPlanner
from docker.deployment_runtime import DeploymentRuntime


def outputs():
    manifest = yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())
    runtime = DeploymentRuntime(manifest)
    scenario_hash = runtime.scenario.scenario_hash
    request = dict(
        scenario_hash=scenario_hash,
        vehicle_id="uav_3",
        request_id="low",
        timestamp=0.0,
        position=[15.0, 15.0],
        heading=0.0,
    )
    runtime.sample(request)
    before = runtime.tick(0.0)
    request.update(vehicle_id="uav_1", request_id="high", timestamp=0.5)
    runtime.sample(request)
    after = runtime.tick(1.0)
    pose = dict(
        scenario_hash=scenario_hash,
        vehicle_id="uav_3",
        timestamp=1.0,
        position=[15.0, 15.0],
        heading=0.0,
        frame_session="visual",
    )
    velocities = []
    for target in (before, after):
        planner = OnboardPlanner(
            manifest, scenario_hash, "uav_3", max_speed=0.5, max_age=2.0
        )
        planner.receive_pose(pose)
        velocities.append(planner.plan(target, 1.0)["velocity"])
    poses = {
        v: dict(pose, vehicle_id=v, position=p)
        for v, p in [("uav_1", [10.0, 15.0]), ("uav_2", [20.0, 15.0])]
    }
    expected = GroundPlannerRuntime(
        scenario_hash, poses, max_speed=0.5, max_age=2.0
    ).plan(after, poses, 1.0)
    errors = []
    for i, vehicle in enumerate(poses):
        planner = OnboardPlanner(
            manifest, scenario_hash, vehicle, max_speed=0.5, max_age=2.0
        )
        for packet in poses.values():
            planner.receive_pose(packet)
        actual = planner.plan(after, 1.0)
        errors.append(
            float(
                np.max(np.abs(np.array(actual["velocity"]) - expected[i]["velocity"]))
            )
        )
    metrics = dict(
        ground_velocity_max_abs_errors=errors,
        aerial_velocities=velocities,
        aerial_velocity_change=float(np.linalg.norm(np.diff(velocities, axis=0))),
        target_max_abs_change=float(
            np.max(np.abs(np.array(after["density"]) - before["density"]))
        ),
        posterior_mean_max_abs_change=float(
            np.max(np.abs(np.array(after["high_mean"]) - before["high_mean"]))
        ),
    )
    return before, after, metrics


def main():
    before, after, metrics = outputs()
    root = Path("docs/assets/sitl")
    root.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    points = np.array(after["query_points"]).reshape((*after["shape"], 2))
    for i, (target, title) in enumerate(
        [(before, "LOW observations only"), (after, "After ground HIGH observation")]
    ):
        image = axes[i].pcolormesh(
            points[:, :, 0],
            points[:, :, 1],
            np.array(target["density"]).reshape(target["shape"]),
            shading="nearest",
        )
        fig.colorbar(image, ax=axes[i], label="Aerial target density")
        v = metrics["aerial_velocities"][i]
        axes[i].quiver(
            15, 15, v[0], v[1], angles="xy", scale_units="xy", scale=0.05, color="red"
        )
        axes[i].set(
            title=title, xlabel="Map north [m]", ylabel="Map east [m]", aspect="equal"
        )
    axes[2].axis("off")
    axes[2].text(
        0.02,
        0.9,
        "Independent onboard planners\n\nGround velocity differences vs\nshared planner: 0, 0 m/s\n\nAerial velocity change:\n"
        + f"{metrics['aerial_velocity_change']:.6g} m/s\n\nCentral MFGP unchanged\nNo simulated vehicle motion",
        va="top",
    )
    fig.savefig(root / "onboard_planning.png", dpi=140)
    (root / "onboard_planning_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    print(json.dumps(metrics))


if __name__ == "__main__":
    main()
