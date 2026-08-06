"""Plot common coverage and runtime metrics from an evaluated comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND,
    load_archive,
)


COLORS = {
    "multifidelity": "#0072B2",
    "egerstedt": "#D55E00",
}
LABELS = {
    "multifidelity": "Proposed multi-fidelity",
    "egerstedt": "Egerstedt baseline",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/baseline_comparison/evaluated.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/baseline_comparison/coverage_metrics.png"),
    )
    return parser.parse_args()


def plot_baseline_comparison_metrics(
    evaluated_path: str | Path,
    output_path: str | Path,
) -> Path:
    """Render coverage histories, summaries, and runtime from saved metrics."""
    metadata, values = load_archive(
        evaluated_path,
        expected_kind=BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND,
    )
    methods, times, coverage, covered_mass, oracle_mass, runtime = (
        _validated_plot_values(values)
    )
    labels = [LABELS.get(method, method.replace("_", " ").title()) for method in methods]
    colors = [COLORS.get(method, f"C{index}") for index, method in enumerate(methods)]

    figure, axes = plt.subplots(2, 2, figsize=(11.8, 8.0), constrained_layout=True)
    coverage_axis, mass_axis, summary_axis, runtime_axis = axes.ravel()

    for method_index, (label, color) in enumerate(zip(labels, colors, strict=True)):
        _plot_episode_history(
            coverage_axis,
            times,
            coverage[method_index],
            label=label,
            color=color,
        )
        _plot_episode_history(
            mass_axis,
            times,
            covered_mass[method_index],
            label=f"{label}: visible",
            color=color,
        )
        oracle = oracle_mass[method_index]
        mass_axis.axhline(
            float(np.mean(oracle)),
            color=color,
            linestyle="--",
            linewidth=1.7,
            label=f"{label}: equal-area oracle",
        )
        if oracle.size > 1:
            lower, upper = np.percentile(oracle, [2.5, 97.5])
            mass_axis.axhspan(lower, upper, color=color, alpha=0.08)

        time_average = np.trapezoid(coverage[method_index], times, axis=1) / (
            times[-1] - times[0]
        )
        final = coverage[method_index, :, -1]
        summary_axis.scatter(
            time_average,
            final,
            color=color,
            s=30,
            alpha=0.5,
        )
        summary_axis.scatter(
            np.mean(time_average),
            np.mean(final),
            color=color,
            edgecolor="white",
            linewidth=0.8,
            marker="D",
            s=85,
            label=f"{label} mean",
            zorder=4,
        )

    coverage_axis.set_title("A. Footprint-normalized coverage (higher is better)")
    coverage_axis.set_xlabel("Mission time [s]")
    coverage_axis.set_ylabel("Coverage effectiveness")
    coverage_axis.set_ylim(-0.02, 1.02)
    coverage_axis.grid(alpha=0.25)
    coverage_axis.legend(fontsize=8)

    mass_axis.set_title("B. Visible hidden-truth mass and equal-area oracle")
    mass_axis.set_xlabel("Mission time [s]")
    mass_axis.set_ylabel("Probability mass")
    mass_upper = max(float(np.max(covered_mass)), float(np.max(oracle_mass)))
    mass_axis.set_ylim(0.0, max(1.0e-12, 1.06 * mass_upper))
    mass_axis.grid(alpha=0.25)
    mass_axis.legend(fontsize=7)

    summary_axis.plot(
        [0.0, 1.0],
        [0.0, 1.0],
        color="#666666",
        linestyle=":",
        linewidth=1.2,
        label="final = time average",
    )
    summary_axis.set_title("C. Episode coverage summary (higher is better)")
    summary_axis.set_xlabel("Time-average coverage")
    summary_axis.set_ylabel("Final coverage")
    summary_axis.set_xlim(-0.02, 1.02)
    summary_axis.set_ylim(-0.02, 1.02)
    summary_axis.set_aspect("equal", adjustable="box")
    summary_axis.grid(alpha=0.25)
    summary_axis.legend(fontsize=7)

    boxplot = runtime_axis.boxplot(
        [runtime[index].ravel() * 1000.0 for index in range(len(methods))],
        tick_labels=labels,
        patch_artist=True,
        showmeans=True,
        meanprops={
            "marker": "D",
            "markerfacecolor": "white",
            "markeredgecolor": "#333333",
            "markersize": 5,
        },
    )
    for patch, color in zip(boxplot["boxes"], colors, strict=True):
        patch.set_facecolor(color)
        patch.set_alpha(0.55)
    runtime_axis.set_title("D. Complete method-step runtime (lower is better)")
    runtime_axis.set_ylabel("Wall-clock time [ms]")
    runtime_axis.tick_params(axis="x", labelrotation=8)
    runtime_axis.grid(axis="y", alpha=0.25)

    fingerprint = metadata.get("scenario_fingerprint", "unknown")
    episode_count = coverage.shape[1]
    figure.suptitle(
        "Paired baseline-comparison metrics\n"
        f"{episode_count} episode{'s' if episode_count != 1 else ''}; "
        f"scenario {str(fingerprint)[:12]}…",
        fontsize=14,
    )
    destination = Path(output_path)
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _plot_episode_history(
    axis,
    times: np.ndarray,
    episodes: np.ndarray,
    *,
    label: str,
    color: str,
) -> None:
    if episodes.shape[0] > 1:
        lower, upper = np.percentile(episodes, [2.5, 97.5], axis=0)
        axis.fill_between(times, lower, upper, color=color, alpha=0.14)
    axis.plot(
        times,
        np.mean(episodes, axis=0),
        color=color,
        linewidth=2.3,
        marker="o" if times.size <= 20 else None,
        markersize=3.5,
        label=label,
    )


def _validated_plot_values(
    values: dict[str, np.ndarray],
) -> tuple[
    list[str],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    methods = [str(value) for value in np.asarray(values["method_names"]).tolist()]
    times = np.asarray(values["state_times"], dtype=float)
    coverage = np.asarray(values["coverage"], dtype=float)
    covered_mass = np.asarray(values["covered_probability_mass"], dtype=float)
    oracle_mass = np.asarray(values["coverage_oracle_mass"], dtype=float)
    runtime = np.asarray(values["runtime_total"], dtype=float)
    if not methods:
        raise ValueError("evaluated archive must contain at least one method")
    if times.ndim != 1 or times.size < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError("state_times must be a strictly increasing vector")
    expected_history = (len(methods), coverage.shape[1], times.size)
    if coverage.ndim != 3 or coverage.shape != expected_history:
        raise ValueError("coverage must have shape (methods, episodes, states)")
    if covered_mass.shape != coverage.shape:
        raise ValueError("covered_probability_mass must match coverage")
    if oracle_mass.shape != coverage.shape[:2]:
        raise ValueError("coverage_oracle_mass must have shape (methods, episodes)")
    if runtime.ndim != 3 or runtime.shape[:2] != coverage.shape[:2]:
        raise ValueError("runtime_total must have shape (methods, episodes, steps)")
    arrays = (times, coverage, covered_mass, oracle_mass, runtime)
    if any(not np.all(np.isfinite(array)) for array in arrays):
        raise ValueError("plot metrics must be finite")
    if np.any(coverage < 0.0) or np.any(coverage > 1.0 + 1.0e-9):
        raise ValueError("coverage must lie in [0, 1]")
    if np.any(covered_mass < 0.0) or np.any(oracle_mass <= 0.0):
        raise ValueError("probability masses must be nonnegative with positive oracles")
    if np.any(runtime < 0.0):
        raise ValueError("runtime values must be nonnegative")
    return methods, times, coverage, covered_mass, oracle_mass, runtime


def main() -> None:
    arguments = parse_args()
    path = plot_baseline_comparison_metrics(arguments.input, arguments.output)
    print(f"baseline_metrics_plot={path}")


if __name__ == "__main__":
    main()
