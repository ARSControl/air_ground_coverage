"""Render an offline MP4 replay from a recorded SITL JSON/JSONL pair."""

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
from matplotlib import animation
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.patheffects as path_effects
import matplotlib.pyplot as plt
import numpy as np
import yaml

from src.core.deployment_scenario import build_scenario
from docker.plot_obstacles import draw_obstacles


def frame_times(duration, *, fps, speed):
    """Recorder offsets, holding the latest available packet at each frame."""
    if (
        not math.isfinite(duration)
        or duration < 0
        or not math.isfinite(fps)
        or fps <= 0
        or not math.isfinite(speed)
        or speed <= 0
    ):
        raise ValueError(
            "duration must be nonnegative; fps and speed must be positive finite values"
        )
    values = np.arange(int(math.floor(duration * fps / speed)) + 1) * speed / fps
    if values[-1] < duration:
        values = np.append(values, duration)
    return values


class MissionReplay:
    """Indexed immutable-input replay; no GP or robot integration is executed."""

    def __init__(self, report, events, manifest):
        self.manifest = manifest
        self.scenario = build_scenario(manifest)
        identity = self.scenario.scenario_hash
        if report["scenario_hash"] != identity:
            raise ValueError(
                "scenario differs from recording; supply original --scenario YAML"
            )
        relevant = sorted(
            (e for e in events if e["topic"] in {"pose", "aerial_target"}),
            key=lambda e: e["received_at"],
        )
        if not any(e["topic"] == "aerial_target" for e in relevant):
            raise ValueError("recording contains no posterior targets")
        if any(not math.isfinite(e["received_at"]) for e in events):
            raise ValueError("invalid recorder timestamp")
        self.start = min(e["received_at"] for e in events)
        self.duration = max(e["received_at"] for e in events) - self.start
        if report.get("timestamp", self.start) < self.start:
            raise ValueError(
                "report predates recorded events; wait for mission completion or use a matching JSON/JSONL pair"
            )
        self.is_excerpt = bool(report.get("recording_excerpt", False))
        self.vehicles = report["vehicles"]
        self.targets, self.target_times = [], []
        self.poses = {vehicle: [] for vehicle in self.vehicles}
        self.pose_times = {vehicle: [] for vehicle in self.vehicles}
        self.grid = None
        for event in relevant:
            packet = event["data"]
            if packet.get("scenario_hash") != identity:
                raise ValueError("recorded packet scenario differs from report")
            timestamp = event["received_at"] - self.start
            if event["topic"] == "aerial_target":
                shape = tuple(packet["shape"])
                points = np.asarray(packet["query_points"], dtype=float)
                if (
                    len(shape) != 2
                    or min(shape) < 2
                    or points.shape != (math.prod(shape), 2)
                    or not np.all(np.isfinite(points))
                ):
                    raise ValueError("invalid posterior grid")
                if self.grid is None:
                    self.grid = points.reshape((*shape, 2))
                if self.grid.shape != (*shape, 2) or not np.array_equal(
                    self.grid.reshape((-1, 2)), points
                ):
                    raise ValueError("posterior grid changed during recording")
                for key in ("high_mean", "high_variance"):
                    field = np.asarray(packet[key], dtype=float)
                    if field.shape != (len(points),) or not np.all(np.isfinite(field)):
                        raise ValueError("invalid posterior field")
                    if key == "high_variance" and np.any(field < 0):
                        raise ValueError("negative posterior variance")
                self.target_times.append(timestamp)
                self.targets.append(packet)
            elif packet.get("vehicle_id") in self.poses:
                position = np.asarray(packet["position"], dtype=float)
                if position.shape != (2,) or not np.all(np.isfinite(position)):
                    raise ValueError("invalid recorded pose")
                vehicle = packet["vehicle_id"]
                self.pose_times[vehicle].append(timestamp)
                self.poses[vehicle].append(position)
        self.poses = {v: np.asarray(p).reshape((-1, 2)) for v, p in self.poses.items()}
        grid = manifest["map"]
        self.north = (
            grid["origin_m"][0]
            + np.arange(self.scenario.high_field.shape[1]) * grid["resolution_m"]
        )
        self.east = (
            grid["origin_m"][1]
            + np.arange(self.scenario.high_field.shape[0]) * grid["resolution_m"]
        )

    def snapshot(self, offset):
        index = bisect_right(self.target_times, offset) - 1
        target = self.targets[index] if index >= 0 else None
        paths = {
            v: p[: bisect_right(self.pose_times[v], offset)]
            for v, p in self.poses.items()
        }
        return target, paths


def render(replay, output, *, fps=10, speed=4, dpi=100):
    """Write an H.264 MP4, final-frame PNG and replay metadata JSON."""
    if output.suffix.lower() != ".mp4":
        raise ValueError("output must have .mp4 extension")
    if not animation.writers.is_available("ffmpeg"):
        raise RuntimeError("ffmpeg is required to export the MP4")
    times = frame_times(replay.duration, fps=fps, speed=speed)
    truth = replay.scenario.high_field
    low = min(float(truth.min()), *(min(t["high_mean"]) for t in replay.targets))
    high = max(float(truth.max()), *(max(t["high_mean"]) for t in replay.targets))
    variance_max = max(max(t["high_variance"]) for t in replay.targets)
    background_cmap = LinearSegmentedColormap.from_list(
        "pale_greys", ["#ffffff", "#c4c4c4"]
    )
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5), layout="constrained")
    image = axes[0].pcolormesh(
        replay.east,
        replay.north,
        truth.T,
        shading="nearest",
        cmap=background_cmap,
        vmin=low,
        vmax=high,
    )
    fig.colorbar(image, ax=axes[0], label="Field value")
    grid = replay.grid
    empty = np.full(grid.shape[:2], np.nan)
    estimate = axes[1].pcolormesh(
        grid[:, :, 1],
        grid[:, :, 0],
        empty,
        shading="nearest",
        cmap=background_cmap,
        vmin=low,
        vmax=high,
    )
    variance = axes[2].pcolormesh(
        grid[:, :, 1],
        grid[:, :, 0],
        empty,
        shading="nearest",
        cmap=background_cmap,
        vmin=0,
        vmax=max(variance_max, 1e-12),
    )
    fig.colorbar(estimate, ax=axes[1], label="Field value")
    fig.colorbar(variance, ax=axes[2], label="Latent variance")
    lines = {}
    for ax, title in zip(
        axes,
        (
            "Ground-truth HIGH field",
            "Recorded HIGH posterior mean",
            "Recorded HIGH posterior variance",
        ),
    ):
        ax.set(
            title=title,
            xlabel="East (m)",
            ylabel="North (m)",
            aspect="equal",
            xlim=(replay.east[0], replay.east[-1]),
            ylim=(replay.north[0], replay.north[-1]),
        )
        draw_obstacles(ax, replay.manifest)
        ax.grid(alpha=0.10)
        for i, (vehicle, values) in enumerate(replay.vehicles.items()):
            color = "#EA580C" if values["role"] == "aerial" else "#15803D"
            (line,) = ax.plot(
                [],
                [],
                color=color,
                linestyle=("-", "--", ":", "-.")[i % 4],
                linewidth=1.8,
                zorder=3,
                path_effects=[
                    path_effects.Stroke(linewidth=3.2, foreground="white"),
                    path_effects.Normal(),
                ],
                label=f"{vehicle} ({values['role']})",
            )
            (marker,) = ax.plot(
                [],
                [],
                marker="^" if values["role"] == "aerial" else "o",
                color=color,
                markersize=6,
                markeredgecolor="white",
                markeredgewidth=0.8,
                zorder=4,
                linestyle="none",
            )
            lines[ax, vehicle] = (line, marker)
    axes[0].legend(fontsize=7, loc="upper left")
    title = fig.suptitle("")

    def update(offset):
        target, paths = replay.snapshot(offset)
        estimate.set_array(
            empty.ravel() if target is None else np.asarray(target["high_mean"])
        )
        variance.set_array(
            empty.ravel() if target is None else np.asarray(target["high_variance"])
        )
        for ax in axes:
            for vehicle, path in paths.items():
                line, marker = lines[ax, vehicle]
                line.set_data(path[:, 1], path[:, 0])
                marker.set_data(path[-1:, 1], path[-1:, 0])
        version = (
            "waiting for posterior"
            if target is None
            else f"posterior v{target['version']}"
        )
        title.set_text(
            f"Recorded mission {'excerpt' if replay.is_excerpt else 'replay'} | recorder +{offset:.1f} s | {version} | {speed:g}× playback"
        )

    # Reserve space for the populated replay title before freezing the layout.
    update(times[0])
    # Lay out the static axes/colorbars once instead of solving layout per frame.
    fig.canvas.draw()
    fig.set_layout_engine("none")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = animation.FFMpegWriter(
        fps=fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p", "-crf", "20"]
    )
    try:
        with writer.saving(fig, str(output), dpi=dpi):
            for index, offset in enumerate(times):
                update(offset)
                writer.grab_frame()
                if index % max(1, int(fps * 10)) == 0:
                    print(f"Rendering frame {index + 1}/{len(times)}", flush=True)
        fig.savefig(output.with_suffix(".png"), dpi=dpi)
    finally:
        plt.close(fig)
    metrics = dict(
        recording_excerpt=replay.is_excerpt,
        recorder_duration_s=replay.duration,
        fps=fps,
        playback_speed=speed,
        frames=len(times),
        encoded_duration_s=len(times) / fps,
        posterior_packets=len(replay.targets),
        pose_packets={v: len(p) for v, p in replay.poses.items()},
        scenario_hash=replay.scenario.scenario_hash,
    )
    output.with_suffix(".replay.json").write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--scenario", type=Path, default=Path("docker/scenarios/sitl_three_drone.yaml")
    )
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument(
        "--speed", type=float, default=4.0, help="Recorder seconds per playback second"
    )
    parser.add_argument("--dpi", type=int, default=100)
    args = parser.parse_args()
    if args.dpi <= 0 or args.dpi % 4:
        parser.error("dpi must be a positive multiple of four for H.264 dimensions")
    report = json.loads(args.report.read_text())
    events = [
        json.loads(line)
        for line in args.report.with_suffix(".events.jsonl").read_text().splitlines()
        if line.strip()
    ]
    manifest = yaml.safe_load(args.scenario.read_text())
    replay = MissionReplay(report, events, manifest)
    print(
        json.dumps(
            render(replay, args.output, fps=args.fps, speed=args.speed, dpi=args.dpi)
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
