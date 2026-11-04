"""Compare FIFO and paper admission on identical real recorded observations.

The posterior is recomputed offline from recording start, without the original
run's unknown pre-recording GP state. This does not rerun vehicle motion.
"""

import argparse
from copy import deepcopy
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
from src.core.observations import Fidelity, Observation
from src.core.multifidelity_estimator import UpdateStatus


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--events",
        type=Path,
        default=Path("output/sitl/mission_faster_aerial.events.jsonl"),
    )
    parser.add_argument(
        "--source-scenario",
        type=Path,
        default=Path("docker/scenarios/sitl_three_drone_1000_samples_role_speeds.yaml"),
    )
    parser.add_argument("--output-stem", default="epistemic_retention")
    args = parser.parse_args()
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    source = yaml.safe_load(args.source_scenario.read_text())
    for key in ("map", "field", "vehicles"):
        if source[key] != manifest[key]:
            raise ValueError("source and active sensing scenario differ")
    identity = build_scenario(source).scenario_hash
    batches = []
    for line in args.events.read_text().splitlines():
        event = json.loads(line)
        if event["topic"] != "sample_response" or not event["data"].get("ok"):
            continue
        data = event["data"]
        if data["scenario_hash"] != identity:
            raise ValueError("source scenario does not match recorded observation hash")
        if data["observations"]:
            batches.append((event["received_at"], data))
    if not batches:
        raise ValueError("no recorded observations")
    origin = min(o["timestamp"] for _, b in batches for o in b["observations"])
    buckets = {}
    for received, batch in batches:
        buckets.setdefault(int(np.floor(received - origin)), []).append(batch)
    first_locations = np.array(
        [o["position"] for _, b in batches[:6] for o in b["observations"]]
    )
    snapshots = []
    metrics = {}
    histories = {}
    for mode in ("newest", "paper_epistemic"):
        config = deepcopy(manifest)
        config["estimator"]["retention"]["policy"] = mode
        if mode == "newest":
            config["estimator"]["retention"].pop("max_total_samples", None)
            config["estimator"]["retention"].update(
                max_low_samples=250, max_high_samples=250, on_full="reject"
            )
        scenario = build_scenario(config)
        estimator = DeploymentEstimator(config, scenario.scenario_hash)
        history = []
        durations = []
        updates = 0
        for step in range(max(buckets) + 1):
            for batch in buckets.get(step, []):
                observations = tuple(
                    Observation(
                        o["timestamp"] - origin,
                        o["robot_id"],
                        tuple(o["position"]),
                        o["value"],
                        Fidelity(o["fidelity"]),
                        o["noise_variance"],
                    )
                    for o in batch["observations"]
                )
                estimator.submit_batch(
                    scenario.scenario_hash,
                    batch["vehicle_id"],
                    batch["request_id"],
                    observations,
                )
            start = perf_counter()
            report = estimator.update(float(step + 1))
            durations.append(perf_counter() - start)
            if report.status is UpdateStatus.FAILED:
                raise RuntimeError(report.message)
            updates += report.status is UpdateStatus.UPDATED
            posterior = estimator.estimator.latest_posterior
            if posterior is not None:
                early = (
                    np.min(
                        np.linalg.norm(
                            posterior.query_points[:, None] - first_locations[None],
                            axis=2,
                        ),
                        axis=1,
                    )
                    <= 3.0
                )
                history.append(
                    (
                        step + 1,
                        posterior.low_sample_count,
                        posterior.high_sample_count,
                        float(posterior.high_variance[early].mean()),
                    )
                )
        posterior = estimator.estimator.latest_posterior
        snapshots.append(posterior)
        histories[mode] = np.array(history)
        metrics[mode] = dict(
            filter=estimator.estimator.filter_diagnostics,
            low_retained=posterior.low_sample_count,
            high_retained=posterior.high_sample_count,
            full_update_max_s=max(durations),
            full_update_median_s=float(np.median(durations)),
            gp_updates=updates,
            early_region_final_variance=history[-1][3],
        )
        print(mode, metrics[mode], flush=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), layout="constrained")
    maximum = max(float(s.high_variance.max()) for s in snapshots)
    for ax, posterior, title in zip(
        axes[0],
        snapshots,
        (
            "Previous newest-first retention",
            f"HIGH-information filter, shared {manifest['estimator']['retention']['max_total_samples']} budget",
        ),
    ):
        points = posterior.query_points.reshape((*posterior.query_shape, 2))
        image = ax.pcolormesh(
            points[:, :, 1],
            points[:, :, 0],
            posterior.high_variance.reshape(posterior.query_shape),
            shading="nearest",
            cmap="Greys",
            vmin=0,
            vmax=maximum,
        )
        ax.scatter(
            first_locations[:, 1],
            first_locations[:, 0],
            s=5,
            c="#2563eb",
            label="Initial sampled locations",
        )
        ax.set(title=title, xlabel="East [m]", ylabel="North [m]", aspect="equal")
        ax.legend(fontsize=7)
        fig.colorbar(image, ax=ax, label="Final HIGH variance")
    for mode, label, color in [
        ("newest", "Newest-first", "#6b7280"),
        ("paper_epistemic", "HIGH-information", "#2563eb"),
    ]:
        h = histories[mode]
        axes[1, 0].plot(
            h[:, 0], h[:, 1], color=color, linestyle="--", label=label + " LOW"
        )
        axes[1, 0].plot(h[:, 0], h[:, 2], color=color, label=label + " HIGH")
        axes[1, 1].plot(h[:, 0], h[:, 3], color=color, label=label)
    axes[1, 0].set(
        title="Retained training observations",
        xlabel="Recorded input time [s]",
        ylabel="Count",
    )
    axes[1, 1].set(
        title="Uncertainty near initial samples",
        xlabel="Recorded input time [s]",
        ylabel="Mean HIGH variance",
    )
    for ax in axes[1]:
        ax.legend(fontsize=8)
        ax.grid(alpha=0.15)
    root = Path("docs/assets/sitl")
    fig.savefig(root / f"{args.output_stem}.png", dpi=140)
    plt.close(fig)
    (root / f"{args.output_stem}_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
