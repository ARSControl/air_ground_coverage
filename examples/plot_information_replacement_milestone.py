"""Deterministic full-budget replacement from actual scenario field outputs."""

import json
from pathlib import Path
from time import perf_counter

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from docker.deployment_runtime import DeploymentRuntime
from docker.deployment_onboard_planner import OnboardPlanner
from src.core.observations import Fidelity, Observation
from src.models.sensors import build_fidelity_fields


def main():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    runtime = DeploymentRuntime(manifest)
    scenario = runtime.scenario
    identity = scenario.scenario_hash
    fields = build_fidelity_fields(
        scenario.high_field,
        manifest["field"]["truth_rho"],
        manifest["field"]["low_smoothing_sigma_cells"],
    )
    rng = np.random.default_rng(1729)
    cells = np.array([(r, c) for r in range(61) for c in range(61)])
    low_cells = cells[rng.choice(len(cells), 498, replace=False)]
    outside = cells[np.linalg.norm(cells - [25, 25], axis=1) > 10]
    high_cells = outside[rng.choice(len(outside), 500, replace=False)]
    low = [
        Observation(
            0.0, "uav_3", (15.0, 15.0), float(fields.low[25, 25]), Fidelity.LOW, 0.04
        ),
        Observation(
            0.0, "uav_3", (15.05, 15.0), float(fields.low[25, 25]), Fidelity.LOW, 0.04
        ),
    ]
    low += [
        Observation(
            0.0,
            "uav_3",
            (float(c - 10), float(r - 10)),
            float(fields.low[r, c]),
            Fidelity.LOW,
            0.04,
        )
        for r, c in low_cells
    ]
    high = [
        Observation(
            0.5,
            "uav_1",
            (float(c - 10), float(r - 10)),
            float(fields.high[r, c]),
            Fidelity.HIGH,
            0.0001,
        )
        for r, c in high_cells
    ]
    runtime.estimator.submit_batch(identity, "uav_3", "initial", low)
    runtime.estimator.submit_batch(identity, "uav_1", "initial", high)
    before = runtime.tick(1.0)
    core = runtime.estimator.estimator
    original = set(core._low_buffer.retained + core._high_buffer.retained)
    assert len(original) == 1000
    runtime.estimator.submit_batch(
        identity,
        "uav_1",
        "new-high",
        (
            Observation(
                2.0,
                "uav_1",
                (15.0, 15.0),
                float(fields.high[25, 25]),
                Fidelity.HIGH,
                0.0001,
            ),
        ),
    )
    start = perf_counter()
    after = runtime.tick(2.0)
    elapsed = perf_counter() - start
    assert after["sample_filter"]["replaced"] == 1
    kept = set(core._low_buffer.retained + core._high_buffer.retained)
    removed = next(iter(original - kept))
    velocities = []
    for target in (before, after):
        planner = OnboardPlanner(
            manifest, identity, "uav_3", max_speed=1.0, max_age=2.0
        )
        planner.receive_pose(
            dict(
                scenario_hash=identity,
                vehicle_id="uav_3",
                timestamp=2.0,
                position=[20.0, 15.0],
                heading=0.0,
                frame_session="visual",
            )
        )
        velocities.append(planner.plan(target, 2.0)["velocity"])
    metrics = dict(
        filter=after["sample_filter"],
        full_replacement_update_s=elapsed,
        removed_fidelity=removed.fidelity.value,
        removed_position=list(removed.position),
        high_mean_max_change=float(
            np.max(np.abs(np.array(after["high_mean"]) - before["high_mean"]))
        ),
        aerial_target_max_change=float(
            np.max(np.abs(np.array(after["density"]) - before["density"]))
        ),
        aerial_velocities=velocities,
        aerial_velocity_change=float(np.linalg.norm(np.diff(velocities, axis=0))),
    )
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
    maximum = max(max(before["high_variance"]), max(after["high_variance"]))
    for ax, target, title in zip(
        axes,
        (before, after),
        ("Before: full 1000-sample budget", "After: one information-based swap"),
    ):
        points = np.array(target["query_points"]).reshape((*target["shape"], 2))
        image = ax.pcolormesh(
            points[:, :, 1],
            points[:, :, 0],
            np.array(target["high_variance"]).reshape(target["shape"]),
            shading="nearest",
            cmap="Greys",
            vmin=0,
            vmax=maximum,
        )
        ax.scatter(
            15,
            15,
            marker="*",
            s=110,
            color="#ea580c",
            label="New HIGH sample",
            zorder=3,
        )
        ax.scatter(
            removed.position[1],
            removed.position[0],
            marker="x",
            s=40,
            color="#2563eb",
            label="Removed " + removed.fidelity.value.upper(),
            zorder=3,
        )
        ax.set(title=title, xlabel="East [m]", ylabel="North [m]", aspect="equal")
        ax.legend(fontsize=7)
        fig.colorbar(image, ax=ax, label="HIGH latent variance")
    d = after["sample_filter"]
    bars = axes[2].bar(
        ["Before", "After"],
        [d["last_replacement_objective_before"], d["last_replacement_objective_after"]],
        color=["#a3a3a3", "#2563eb"],
    )
    axes[2].set(title="Grid-weighted HIGH uncertainty", ylabel="Mean HIGH variance")
    axes[2].bar_label(bars, fmt="%.6f", padding=3)
    root = Path("docs/assets/sitl")
    fig.savefig(root / "information_replacement.png", dpi=140)
    plt.close(fig)
    (root / "information_replacement_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
