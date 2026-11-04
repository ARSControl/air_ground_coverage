"""Render a recorded SITL readiness run from its real JSON/JSONL outputs."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from src.core.deployment_scenario import build_scenario
from docker.plot_obstacles import draw_obstacles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("docker/scenarios/sitl_three_drone.yaml"),
        help="Original scenario YAML; its hash must match the recorded run",
    )
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    events = [
        json.loads(line)
        for line in args.report.with_suffix(".events.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    manifest = yaml.safe_load(args.scenario.read_text(encoding="utf-8"))
    fig = render_summary(report, events, manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


def render_summary(report, events, manifest):
    """Build a figure from recorded outputs and hash-verified scenario truth."""
    scenario = build_scenario(manifest)
    if scenario.scenario_hash != report["scenario_hash"]:
        raise ValueError(
            "scenario hash differs from recorded run; supply its original --scenario YAML"
        )
    start = min((e["received_at"] for e in events), default=report["timestamp"])
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    latest = next(
        (e["data"] for e in reversed(events) if e["topic"] == "aerial_target"), None
    )
    if latest is not None:
        points = np.asarray(latest["query_points"]).reshape((*latest["shape"], 2))
        image = axes[0, 0].pcolormesh(
            points[:, :, 1],
            points[:, :, 0],
            np.asarray(latest["density"]).reshape(latest["shape"]),
            shading="nearest",
            cmap="viridis",
        )
        fig.colorbar(image, ax=axes[0, 0], label="Aerial target density")
        for ax, key, title, cmap in (
            (axes[0, 2], "high_mean", "Final HIGH posterior mean", "viridis"),
            (axes[1, 2], "high_variance", "Final HIGH posterior variance", "Greys"),
        ):
            field = ax.pcolormesh(
                points[:, :, 1],
                points[:, :, 0],
                np.asarray(latest[key]).reshape(latest["shape"]),
                shading="nearest",
                cmap=cmap,
                **({"vmin": 0} if key == "high_variance" else {}),
            )
            fig.colorbar(
                field,
                ax=ax,
                label="Field value" if key == "high_mean" else "Latent variance",
            )
            ax.set(title=title, xlabel="East (m)", ylabel="North (m)", aspect="equal")
    axes[0, 0].set(
        title="Recorded central MFGP aerial target",
        xlabel="East (m)",
        ylabel="North (m)",
        aspect="equal",
    )
    grid = manifest["map"]
    north = (
        grid["origin_m"][0]
        + np.arange(scenario.high_field.shape[1]) * grid["resolution_m"]
    )
    east = (
        grid["origin_m"][1]
        + np.arange(scenario.high_field.shape[0]) * grid["resolution_m"]
    )
    axes[0, 1].pcolormesh(
        east,
        north,
        scenario.high_field.T,
        shading="nearest",
        cmap="Greys",
        vmin=0,
        alpha=0.22,
        zorder=0,
    )
    for vehicle, values in report["vehicles"].items():
        path = [
            e["data"]["position"]
            for e in events
            if e["topic"] == "pose" and e["data"].get("vehicle_id") == vehicle
        ]
        if path:
            positions = np.asarray(path)
            (line,) = axes[0, 1].plot(
                positions[:, 1], positions[:, 0], label=f"{vehicle} ({values['role']})"
            )
            axes[0, 1].scatter(
                positions[-1, 1],
                positions[-1, 0],
                s=35,
                color=line.get_color(),
                zorder=3,
            )
    axes[0, 1].set(
        title="Robot trajectories over ground truth",
        xlabel="East (m)",
        ylabel="North (m)",
        aspect="equal",
    )
    if axes[0, 1].lines:
        axes[0, 1].legend(fontsize=8)
    statuses = [e for e in events if e["topic"] == "status"]
    for key, label in (
        ("low_sample_count", "LOW retained"),
        ("high_sample_count", "HIGH retained"),
    ):
        axes[1, 0].plot(
            [e["received_at"] - start for e in statuses],
            [e["data"].get(key, 0) for e in statuses],
            label=label,
        )
    axes[1, 0].set(
        title="Central retained observations",
        xlabel="Recorder time (s)",
        ylabel="Count",
    )
    axes[1, 0].legend()
    truth = axes[1, 1].pcolormesh(
        east, north, scenario.high_field.T, shading="nearest", cmap="viridis"
    )
    fig.colorbar(truth, ax=axes[1, 1], label="Field value")
    axes[1, 1].set(
        title="Ground-truth HIGH field",
        xlabel="East (m)",
        ylabel="North (m)",
        aspect="equal",
    )
    for ax in (axes[0, 0], axes[0, 1], axes[0, 2], axes[1, 1], axes[1, 2]):
        draw_obstacles(ax, manifest)
    axes[0, 1].legend(fontsize=7)
    for ax in axes.ravel():
        ax.grid(alpha=0.2)
    commands = sum(s["received"] for s in report.get("command_stats", {}).values())
    fig.suptitle(
        f"SITL pipeline ready: {report['ready']} | observed vehicle commands: {commands}"
    )
    fig.tight_layout()
    return fig


if __name__ == "__main__":
    main()
