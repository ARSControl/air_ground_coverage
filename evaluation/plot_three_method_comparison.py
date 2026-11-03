"""Plot common metrics and trajectories for the paired three-method study."""

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
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    THREE_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    scenario_fingerprint,
)


METHODS = ("multifidelity", "egerstedt", "zhang")
LABELS = {
    "multifidelity": "Proposed multi-fidelity",
    "egerstedt": "Rudolph [17]",
    "zhang": "Zhang",
}
COLORS = {"multifidelity": "#0072B2", "egerstedt": "#D55E00", "zhang": "#009E73"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/three_method_comparison")
    parser.add_argument("--evaluated", type=Path, default=root / "evaluated.npz")
    parser.add_argument(
        "--multifidelity", type=Path, default=root / "multifidelity/raw.npz"
    )
    parser.add_argument("--egerstedt", type=Path, default=root / "egerstedt/raw.npz")
    parser.add_argument("--zhang", type=Path, default=root / "zhang/raw.npz")
    parser.add_argument("--output-dir", "-o", type=Path, default=root / "plots")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def plot_three_method_comparison(
    evaluated_path: str | Path,
    multifidelity_path: str | Path,
    egerstedt_path: str | Path,
    zhang_path: str | Path,
    output_directory: str | Path,
    *,
    episode: int = 0,
    dpi: int = 180,
) -> tuple[Path, Path]:
    """Write aggregate metrics and a common-scale three-panel trajectory figure."""
    evaluated_metadata, evaluated = load_archive(
        evaluated_path,
        expected_kind=THREE_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    )
    raw_metadata: list[dict[str, object]] = []
    raw: list[dict[str, np.ndarray]] = []
    for path, kind in (
        (multifidelity_path, MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND),
        (egerstedt_path, EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND),
        (zhang_path, ZHANG_COMPARISON_RAW_ARCHIVE_KIND),
    ):
        metadata, values = load_archive(path, expected_kind=kind)
        raw_metadata.append(metadata)
        raw.append(values)
    fingerprint = _validate(evaluated_metadata, raw_metadata, raw, episode)
    labels = dict(LABELS)
    zhang_sensing = raw_metadata[2].get("aerial_sensing", {})
    if isinstance(zhang_sensing, dict) and zhang_sensing.get("mode") == "limited":
        labels["zhang"] = "Zhang-LR"
    if isinstance(dpi, bool) or not isinstance(dpi, int) or dpi < 72:
        raise ValueError("dpi must be an integer of at least 72")
    output_root = Path(output_directory)
    metrics_path = output_root / "metrics.png"
    trajectories_path = output_root / f"trajectories_episode_{episode:03d}.png"
    output_root.mkdir(parents=True, exist_ok=True)
    _plot_metrics(evaluated, metrics_path, fingerprint, dpi, labels)
    _plot_trajectories(raw, episode, trajectories_path, dpi, labels)
    return metrics_path, trajectories_path


def _plot_metrics(
    values: dict[str, np.ndarray],
    output: Path,
    fingerprint: str,
    dpi: int,
    labels: dict[str, str],
) -> None:
    times = values["state_times"]
    figure, axes = plt.subplots(2, 3, figsize=(15.2, 8.6), constrained_layout=True)
    coverage_axis, cost_axis, travel_axis, summary_axis, runtime_axis, phase_axis = (
        axes.ravel()
    )
    for index, method in enumerate(METHODS):
        label = labels[method]
        color = COLORS[method]
        _history(
            coverage_axis, times, values["coverage"][index], label=label, color=color
        )
        _history(
            cost_axis,
            times,
            values["locational_cost"][index],
            label=label,
            color=color,
        )
        _history(
            travel_axis,
            times,
            values["ground_travel_distance"][index],
            label=label,
            color=color,
        )
    coverage_axis.set_title("A. Footprint-normalized coverage")
    coverage_axis.set_ylabel("effectiveness")
    cost_axis.set_title("B. Locational cost")
    cost_axis.set_ylabel(r"$H$")
    travel_axis.set_title("C. Accumulated ground travel")
    travel_axis.set_ylabel("distance")
    for axis in (coverage_axis, cost_axis, travel_axis):
        axis.set_xlabel("mission time [s]")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)

    x = np.arange(len(METHODS), dtype=float)
    width = 0.34
    final_mean = np.mean(values["final_coverage"], axis=1)
    average_mean = np.mean(values["time_average_coverage"], axis=1)
    final_ci = values["final_coverage_ci"]
    average_ci = values["time_average_coverage_ci"]
    summary_axis.bar(
        x - width / 2,
        final_mean,
        width,
        yerr=np.vstack((final_mean - final_ci[:, 0], final_ci[:, 1] - final_mean)),
        label="final",
        color=[COLORS[method] for method in METHODS],
        alpha=0.85,
        capsize=3,
    )
    summary_axis.bar(
        x + width / 2,
        average_mean,
        width,
        yerr=np.vstack(
            (average_mean - average_ci[:, 0], average_ci[:, 1] - average_mean)
        ),
        label="time average",
        color=[COLORS[method] for method in METHODS],
        alpha=0.4,
        capsize=3,
    )
    summary_axis.set_xticks(x, [labels[method] for method in METHODS], rotation=8)
    summary_axis.set_ylabel("coverage effectiveness")
    summary_axis.set_title("D. Paired coverage summary (95% bootstrap CI)")
    summary_axis.legend(fontsize=8)
    summary_axis.grid(axis="y", alpha=0.25)

    boxplot = runtime_axis.boxplot(
        [values["runtime_total"][index].ravel() * 1000.0 for index in range(3)],
        tick_labels=[labels[method] for method in METHODS],
        patch_artist=True,
        showmeans=True,
    )
    for patch, method in zip(boxplot["boxes"], METHODS, strict=True):
        patch.set_facecolor(COLORS[method])
        patch.set_alpha(0.55)
    runtime_axis.set_title("E. Complete controller-step runtime")
    runtime_axis.set_ylabel("wall time [ms]")
    runtime_axis.tick_params(axis="x", labelrotation=8)
    runtime_axis.grid(axis="y", alpha=0.25)

    transition = values["zhang_phase_transition_time"]
    support = values.get("zhang_aerial_cell_support_fraction")
    if labels["zhang"] == "Zhang-LR" and support is not None:
        minimum_support = np.min(support, axis=2)
        mean_support = np.mean(support, axis=2)
        _history(
            phase_axis,
            times,
            minimum_support,
            label="minimum cell support",
            color="#7F3C8D",
        )
        _history(
            phase_axis,
            times,
            mean_support,
            label="mean cell support",
            color=COLORS["zhang"],
        )
        finite = transition[np.isfinite(transition)]
        transition_text = (
            "deployment unfinished"
            if finite.size == 0
            else f"Algorithm 2 starts: {np.mean(finite):.2f} s"
        )
        phase_axis.text(
            0.03,
            0.04,
            transition_text,
            transform=phase_axis.transAxes,
            va="bottom",
        )
        phase_axis.set_title("F. Zhang-LR sensed power-cell support")
        phase_axis.set_ylabel("area fraction")
        phase_axis.set_ylim(0.0, 1.05)
        phase_axis.legend(fontsize=8)
    else:
        finite = transition[np.isfinite(transition)]
        if finite.size:
            phase_axis.hist(
                finite, bins=min(10, finite.size), color=COLORS["zhang"], alpha=0.75
            )
        unfinished = int(np.count_nonzero(~np.isfinite(transition)))
        phase_axis.text(
            0.03,
            0.96,
            f"unfinished deployments: {unfinished}/{transition.size}",
            transform=phase_axis.transAxes,
            va="top",
        )
        phase_axis.set_title("F. Zhang Algorithm 1 transition time")
        phase_axis.set_ylabel("episodes")
    phase_axis.set_xlabel("mission time [s]")
    phase_axis.grid(axis="y", alpha=0.25)
    figure.suptitle(
        f"Paired A/G baseline comparison — scenario {fingerprint[:12]}…",
        fontsize=14,
    )
    figure.savefig(output, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(figure)


def _plot_trajectories(
    raw: list[dict[str, np.ndarray]],
    episode: int,
    output: Path,
    dpi: int,
    labels: dict[str, str],
) -> None:
    points = raw[0]["query_points"]
    x_values = np.unique(points[:, 0])
    y_values = np.unique(points[:, 1])
    truth = raw[0]["truth_density"][episode].reshape(y_values.size, x_values.size)
    extent = (x_values[0], x_values[-1], y_values[0], y_values[-1])
    normalization = Normalize(vmin=0.0, vmax=float(np.max(truth)))
    aerial = (
        raw[0]["aerial_states"][episode, :, :, :2],
        raw[1]["aerial_positions"][episode],
        raw[2]["aerial_positions"][episode],
    )
    ground = (
        raw[0]["ground_states"][episode, :, :, :2],
        raw[1]["ground_positions"][episode],
        raw[2]["ground_positions"][episode],
    )
    figure, axes = plt.subplots(1, 3, figsize=(16.2, 5.4), constrained_layout=True)
    image = None
    for index, (axis, method) in enumerate(zip(axes, METHODS, strict=True)):
        image = axis.imshow(
            truth,
            origin="lower",
            extent=extent,
            cmap="magma",
            norm=normalization,
            interpolation="nearest",
            aspect="equal",
        )
        for robot in range(aerial[index].shape[1]):
            axis.plot(
                aerial[index][:, robot, 0],
                aerial[index][:, robot, 1],
                linestyle="--",
                color="tab:cyan",
                linewidth=1.1,
                alpha=0.8,
            )
            if method == "zhang" and labels[method] == "Zhang-LR":
                axis.add_patch(
                    Circle(
                        aerial[index][-1, robot],
                        radius=float(raw[index]["aerial_sensing_range"]),
                        fill=False,
                        edgecolor="cyan",
                        linestyle="--",
                        linewidth=0.65,
                        alpha=0.55,
                    )
                )
        for robot in range(ground[index].shape[1]):
            axis.plot(
                ground[index][:, robot, 0],
                ground[index][:, robot, 1],
                color="tab:green",
                linewidth=1.2,
                alpha=0.8,
            )
            axis.add_patch(
                Circle(
                    ground[index][-1, robot],
                    radius=float(raw[index]["ground_sensing_range"]),
                    fill=False,
                    edgecolor="white",
                    linewidth=0.65,
                    alpha=0.65,
                )
            )
        axis.scatter(
            aerial[index][0, :, 0], aerial[index][0, :, 1], marker="x", color="cyan"
        )
        axis.scatter(
            ground[index][0, :, 0], ground[index][0, :, 1], marker="x", color="lime"
        )
        axis.scatter(
            aerial[index][-1, :, 0],
            aerial[index][-1, :, 1],
            marker="^",
            color="cyan",
            edgecolor="black",
            linewidth=0.4,
        )
        axis.scatter(
            ground[index][-1, :, 0],
            ground[index][-1, :, 1],
            marker="o",
            color="lime",
            edgecolor="black",
            linewidth=0.4,
        )
        title = labels[method]
        if method == "zhang":
            transition = float(raw[index]["phase_transition_time"][episode])
            title += (
                "\nAlgorithm 2 not reached"
                if not np.isfinite(transition)
                else f"\nAlgorithm 2 starts at {transition:.2f} s"
            )
        axis.set_title(title)
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
    if image is not None:
        figure.colorbar(image, ax=axes, shrink=0.78, label="hidden-truth density")
    figure.suptitle(f"Paired trajectories, episode {episode}", fontsize=14)
    figure.savefig(output, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(figure)


def _history(axis, times, episodes, *, label: str, color: str) -> None:
    if episodes.shape[0] > 1:
        lower, upper = np.percentile(episodes, (2.5, 97.5), axis=0)
        axis.fill_between(times, lower, upper, color=color, alpha=0.14)
    axis.plot(times, np.mean(episodes, axis=0), color=color, label=label, linewidth=2.0)


def _validate(
    evaluated_metadata: dict[str, object],
    raw_metadata: list[dict[str, object]],
    raw: list[dict[str, np.ndarray]],
    episode: int,
) -> str:
    if isinstance(episode, bool) or not isinstance(episode, int) or episode < 0:
        raise ValueError("episode must be a nonnegative integer")
    fingerprints = [scenario_fingerprint(values) for values in raw]
    recorded = [metadata.get("scenario_fingerprint") for metadata in raw_metadata]
    evaluated = evaluated_metadata.get("scenario_fingerprint")
    if len(set((*fingerprints, *recorded, evaluated))) != 1:
        raise ValueError("plots require one paired scenario fingerprint")
    if episode >= raw[0]["seeds"].size:
        raise IndexError("episode is outside the paired archive")
    return fingerprints[0]


def main() -> None:
    arguments = parse_args()
    metrics, trajectories = plot_three_method_comparison(
        arguments.evaluated,
        arguments.multifidelity,
        arguments.egerstedt,
        arguments.zhang,
        arguments.output_dir,
        episode=arguments.episode,
        dpi=arguments.dpi,
    )
    print(f"three_method_metrics_plot={metrics}")
    print(f"three_method_trajectory_plot={trajectories}")


if __name__ == "__main__":
    main()
