"""Compare deployed 250/1000 retention caps on identical deterministic field data."""

import json
from pathlib import Path
from time import perf_counter

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from src.core.deployment_estimator import DeploymentEstimator
from src.core.deployment_scenario import build_scenario
from src.core.multifidelity_estimator import UpdateStatus
from src.core.observations import Fidelity, Observation
from src.models.sensors import build_fidelity_fields


def main():
    rng = np.random.default_rng(123)
    near = np.array([(row, col) for row in range(21) for col in range(21)])
    distant = np.array([(row, col) for row in range(35, 61) for col in range(35, 61)])
    # 676 unique distant cells plus the remainder from the eastern strip.
    distant = np.vstack(
        (distant, np.array([(row, col) for row in range(35) for col in range(35, 61)]))
    )
    cells = np.vstack(
        (
            near[rng.choice(len(near), 250, replace=False)],
            distant[rng.choice(len(distant), 750, replace=False)],
        )
    )
    results, snapshots = {}, []
    for filename in ("sitl_three_drone_250_samples.yaml", "sitl_three_drone.yaml"):
        manifest = yaml.safe_load((Path("docker/scenarios") / filename).read_text())
        scenario = build_scenario(manifest)
        fields = build_fidelity_fields(
            scenario.high_field,
            manifest["field"]["truth_rho"],
            manifest["field"]["low_smoothing_sigma_cells"],
        )
        estimator = DeploymentEstimator(manifest, scenario.scenario_hash)
        for phase, selected in enumerate((cells[:250], cells[250:])):
            for vehicle, fidelity, raster, noise in (
                ("uav_3", Fidelity.LOW, fields.low, 0.04),
                ("uav_1", Fidelity.HIGH, fields.high, 0.0001),
            ):
                batch = tuple(
                    Observation(
                        float(phase),
                        vehicle,
                        (float(col - 10), float(row - 10)),
                        float(raster[row, col]),
                        fidelity,
                        noise,
                    )
                    for row, col in selected
                )
                estimator.submit_batch(
                    scenario.scenario_hash, vehicle, str(phase), batch
                )
            start = perf_counter()
            assert estimator.update(float(phase)).status is UpdateStatus.UPDATED
            update_duration = perf_counter() - start
            snapshot = estimator.estimator.latest_posterior
            if phase == 0:
                initial_variance = snapshot.high_variance.copy()
        cap = manifest["estimator"]["retention"]["max_high_samples"]
        early = np.all(snapshot.query_points <= 10, axis=1)
        results[str(cap)] = dict(
            low_retained=snapshot.low_sample_count,
            high_retained=snapshot.high_sample_count,
            early_region_variance_before=float(initial_variance[early].mean()),
            early_region_variance_after=float(snapshot.high_variance[early].mean()),
            update_duration_s=update_duration,
            fit_duration_s=snapshot.fit_duration,
            prediction_duration_s=snapshot.prediction_duration,
        )
        snapshots.append(snapshot)
    root = Path("docs/assets/sitl")
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), layout="constrained")
    maximum = max(float(s.high_variance.max()) for s in snapshots)
    for ax, snapshot, cap in zip(axes[:2], snapshots, (250, 1000)):
        points = snapshot.query_points.reshape((*snapshot.query_shape, 2))
        mesh = ax.pcolormesh(
            points[:, :, 1],
            points[:, :, 0],
            snapshot.high_variance.reshape(snapshot.query_shape),
            shading="nearest",
            cmap="Greys",
            vmin=0,
            vmax=maximum,
        )
        ax.set(
            title=f"{cap} samples per fidelity",
            xlabel="East [m]",
            ylabel="North [m]",
            aspect="equal",
        )
        ax.scatter(
            cells[:250, 0] - 10,
            cells[:250, 1] - 10,
            s=1,
            color="tab:blue",
            label="First 250 locations",
        )
        ax.legend(fontsize=7)
        fig.colorbar(mesh, ax=ax, label="Final HIGH variance")
    axes[2].bar(
        ["250 cap", "1000 cap"],
        [results[str(c)]["early_region_variance_after"] for c in (250, 1000)],
    )
    axes[2].set(title="Previously sampled region", ylabel="Mean final HIGH variance")
    fig.savefig(root / "retention_comparison.png", dpi=140)
    plt.close(fig)
    (root / "retention_comparison_metrics.json").write_text(
        json.dumps(results, indent=2) + "\n"
    )
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
