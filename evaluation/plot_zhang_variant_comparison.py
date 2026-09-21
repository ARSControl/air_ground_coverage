"""Plot a direct paired standard-Zhang versus Zhang-LR comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import Normalize  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402
import numpy as np  # noqa: E402

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    ZHANG_VARIANT_COMPARISON_EVALUATED_ARCHIVE_KIND,
    load_archive,
    scenario_fingerprint,
)


COLORS = {"standard": "#0072B2", "lr": "#D55E00"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/zhang_variant_comparison")
    parser.add_argument("--evaluated", type=Path, default=root / "evaluated.npz")
    parser.add_argument("--standard", type=Path, default=root / "standard/raw.npz")
    parser.add_argument("--lr", type=Path, default=root / "lr/raw.npz")
    parser.add_argument("--output", "-o", type=Path, default=root / "comparison.png")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def plot_zhang_variant_comparison(
    evaluated_path: str | Path,
    standard_path: str | Path,
    lr_path: str | Path,
    output_path: str | Path,
    *,
    episode: int = 0,
    dpi: int = 180,
) -> Path:
    """Render trajectories, divergence, support, mass, and coverage directly."""
    evaluated_metadata, evaluated = load_archive(
        evaluated_path,
        expected_kind=ZHANG_VARIANT_COMPARISON_EVALUATED_ARCHIVE_KIND,
    )
    standard_metadata, standard = load_archive(
        standard_path, expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND
    )
    lr_metadata, limited = load_archive(
        lr_path, expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND
    )
    fingerprint = _validate(
        evaluated_metadata,
        standard_metadata,
        standard,
        lr_metadata,
        limited,
        episode,
    )
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 3, figsize=(15.8, 9.2), constrained_layout=True)
    _trajectory_panel(axes[0, 0], standard, episode, "Standard Zhang", limited=False)
    _trajectory_panel(axes[0, 1], limited, episode, "Zhang-LR", limited=True)
    _difference_panel(axes[0, 2], evaluated, episode)
    _support_panel(axes[1, 0], evaluated, episode)
    _target_mass_panel(axes[1, 1], evaluated, episode)
    _coverage_panel(axes[1, 2], evaluated, episode)
    figure.suptitle(
        "Direct paired Zhang comparison — "
        f"seed {int(evaluated['seeds'][episode])}, scenario {fingerprint[:12]}…",
        fontsize=14,
    )
    figure.savefig(output, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return output


def _trajectory_panel(
    axis, values: dict[str, np.ndarray], episode: int, title: str, *, limited: bool
) -> None:
    points = values["query_points"]
    xs = np.unique(points[:, 0])
    ys = np.unique(points[:, 1])
    truth = values["truth_density"][episode].reshape(ys.size, xs.size)
    extent = (xs[0], xs[-1], ys[0], ys[-1])
    axis.imshow(
        truth,
        origin="lower",
        extent=extent,
        cmap="magma",
        norm=Normalize(vmin=0.0, vmax=float(np.max(truth))),
        interpolation="nearest",
        aspect="equal",
    )
    aerial = values["aerial_positions"][episode]
    ground = values["ground_positions"][episode]
    for robot in range(aerial.shape[1]):
        axis.plot(
            aerial[:, robot, 0],
            aerial[:, robot, 1],
            "--",
            color="cyan",
            linewidth=1.2,
            alpha=0.8,
        )
        if limited:
            axis.add_patch(
                Circle(
                    aerial[-1, robot],
                    radius=float(values["aerial_sensing_range"]),
                    fill=False,
                    edgecolor="cyan",
                    linestyle="--",
                    linewidth=0.8,
                    alpha=0.65,
                )
            )
    for robot in range(ground.shape[1]):
        axis.plot(
            ground[:, robot, 0],
            ground[:, robot, 1],
            color="lime",
            linewidth=1.2,
            alpha=0.8,
        )
    axis.scatter(aerial[0, :, 0], aerial[0, :, 1], marker="x", color="cyan")
    axis.scatter(ground[0, :, 0], ground[0, :, 1], marker="x", color="lime")
    axis.scatter(
        aerial[-1, :, 0],
        aerial[-1, :, 1],
        marker="^",
        color="cyan",
        edgecolor="black",
        linewidth=0.4,
    )
    axis.scatter(
        ground[-1, :, 0],
        ground[-1, :, 1],
        marker="o",
        color="lime",
        edgecolor="black",
        linewidth=0.4,
    )
    transition = float(values["phase_transition_time"][episode])
    axis.set_title(f"{title}\nAlgorithm 2 starts at {transition:.2f} s")
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.set_xlim(extent[0], extent[1])
    axis.set_ylim(extent[2], extent[3])


def _difference_panel(axis, values: dict[str, np.ndarray], episode: int) -> None:
    times = values["state_times"]
    aerial = values["aerial_position_difference"][episode]
    ground = values["ground_position_difference"][episode]
    axis.plot(times, np.max(aerial, axis=1), color="tab:cyan", label="max aerial")
    axis.plot(
        times, np.mean(aerial, axis=1), "--", color="tab:cyan", label="mean aerial"
    )
    axis.plot(times, np.max(ground, axis=1), color="tab:green", label="max ground")
    axis.plot(
        times,
        np.mean(ground, axis=1),
        "--",
        color="tab:green",
        label="mean ground",
    )
    transition = float(values["phase_transition_time"][0, episode])
    axis.axvline(transition, color="black", linestyle=":", alpha=0.65)
    axis.set_title("Position divergence: standard vs LR")
    axis.set_xlabel("mission time [s]")
    axis.set_ylabel("Euclidean distance")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)


def _support_panel(axis, values: dict[str, np.ndarray], episode: int) -> None:
    times = values["state_times"]
    support = values["cell_support_fraction"][1, episode]
    axis.plot(times, np.min(support, axis=1), color="#7F3C8D", label="minimum support")
    axis.plot(times, np.mean(support, axis=1), color=COLORS["lr"], label="mean support")
    axis.set_title("Zhang-LR sensed power-cell fraction")
    axis.set_xlabel("mission time [s]")
    axis.set_ylabel("area fraction")
    axis.set_ylim(0.0, 1.05)
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)


def _target_mass_panel(axis, values: dict[str, np.ndarray], episode: int) -> None:
    masses = values["target_masses"][:, episode]
    robots = np.arange(masses.shape[1])
    width = 0.38
    axis.bar(
        robots - width / 2,
        masses[0],
        width,
        color=COLORS["standard"],
        label="standard",
    )
    axis.bar(
        robots + width / 2,
        masses[1],
        width,
        color=COLORS["lr"],
        label="Zhang-LR",
    )
    axis.set_xticks(robots, [f"A{index}" for index in robots])
    axis.set_title("Consensus target cell masses")
    axis.set_ylabel("LOW-field mass")
    axis.grid(axis="y", alpha=0.25)
    axis.legend(fontsize=8)


def _coverage_panel(axis, values: dict[str, np.ndarray], episode: int) -> None:
    times = values["state_times"]
    standard = values["coverage"][0, episode]
    limited = values["coverage"][1, episode]
    axis.plot(times, standard, color=COLORS["standard"], label="standard Zhang")
    axis.plot(times, limited, color=COLORS["lr"], label="Zhang-LR")
    axis.text(
        0.03,
        0.96,
        f"final: {standard[-1]:.4f} vs {limited[-1]:.4f}",
        transform=axis.transAxes,
        va="top",
    )
    axis.set_title("Footprint-normalized ground coverage")
    axis.set_xlabel("mission time [s]")
    axis.set_ylabel("effectiveness")
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)


def _validate(
    evaluated_metadata: dict[str, object],
    standard_metadata: dict[str, object],
    standard: dict[str, np.ndarray],
    lr_metadata: dict[str, object],
    limited: dict[str, np.ndarray],
    episode: int,
) -> str:
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    fingerprints = (
        scenario_fingerprint(standard),
        scenario_fingerprint(limited),
        standard_metadata.get("scenario_fingerprint"),
        lr_metadata.get("scenario_fingerprint"),
        evaluated_metadata.get("scenario_fingerprint"),
    )
    if len(set(fingerprints)) != 1:
        raise ValueError("plot inputs do not share one paired scenario")
    if episode >= standard["seeds"].size:
        raise IndexError("episode is outside the paired archives")
    return fingerprints[0]


def main() -> None:
    arguments = parse_args()
    output = plot_zhang_variant_comparison(
        arguments.evaluated,
        arguments.standard,
        arguments.lr,
        arguments.output,
        episode=arguments.episode,
        dpi=arguments.dpi,
    )
    print(f"zhang_variant_plot={output}")


if __name__ == "__main__":
    main()
