"""Plot evaluated ablation metrics without importing simulation code."""

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

from evaluation.multifidelity_ablation_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    load_archive,
)


COLORS = {
    "full": "#0072B2",
    "no_discrepancy": "#D55E00",
    "no_high_to_aerial": "#009E73",
    "no_uncertainty": "#CC79A7",
}
LABELS = {
    "full": "Full method",
    "no_discrepancy": "No discrepancy",
    "no_high_to_aerial": "No HIGH-to-aerial",
    "no_uncertainty": "No uncertainty target",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_ablation_evaluated.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_ablation_metrics.png"),
    )
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    return parser.parse_args()


def plot_ablation_evaluation(
    evaluated_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 1000,
) -> Path:
    """Render the agreed four-panel ablation history figure."""
    if isinstance(bootstrap_samples, bool) or bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be a positive integer")
    _, values = load_archive(evaluated_path, expected_kind=EVALUATED_ARCHIVE_KIND)
    variants = [str(item) for item in values["variant_names"].tolist()]
    figure, axes = plt.subplots(2, 2, figsize=(11.5, 8.0), constrained_layout=True)
    rng = np.random.default_rng(20260802)
    posterior_panels = (
        (axes[0, 0], "kl", "KL divergence", "lower is better"),
        (axes[0, 1], "nrmse", "NRMSE of latent HIGH field", "lower is better"),
        (
            axes[1, 0],
            "calibration_95",
            "Empirical 95% calibration",
            "target = 0.95",
        ),
    )
    for axis, metric, title, direction in posterior_panels:
        _plot_posterior_metric(axis, values, variants, metric, rng, bootstrap_samples)
        axis.set_title(f"{title}\n({direction})")
        axis.set_xlabel("Mission time [s]")
        axis.grid(True, alpha=0.25)
    axes[1, 0].axhline(
        0.95, color="#333333", linestyle="--", linewidth=1.2, label="Ideal 0.95"
    )
    axes[1, 0].set_ylim(-0.02, 1.02)

    coverage_axis = axes[1, 1]
    for variant_index, variant in enumerate(variants):
        series = values["coverage"][variant_index]
        center, lower, upper = _bootstrap_band(series, rng, bootstrap_samples)
        color = COLORS.get(variant, f"C{variant_index}")
        coverage_axis.plot(
            values["state_times"],
            center,
            color=color,
            linewidth=2.0,
            label=LABELS.get(variant, variant),
        )
        coverage_axis.fill_between(
            values["state_times"], lower, upper, color=color, alpha=0.16
        )
    coverage_axis.set_title(
        "Footprint-normalized coverage effectiveness\n(higher is better)"
    )
    coverage_axis.set_xlabel("Mission time [s]")
    coverage_axis.set_ylim(-0.02, 1.02)
    coverage_axis.grid(True, alpha=0.25)

    handles, labels = coverage_axis.get_legend_handles_labels()
    calibration_handles, calibration_labels = axes[1, 0].get_legend_handles_labels()
    if calibration_handles:
        handles += calibration_handles[-1:]
        labels += calibration_labels[-1:]
    figure.legend(
        handles,
        labels,
        loc="outside lower center",
        ncol=min(5, len(handles)),
        frameon=False,
    )
    figure.suptitle(
        "Multi-fidelity closed-loop ablation study\n"
        "lines: episode mean; bands: episode-bootstrap 95% interval",
        fontsize=14,
    )
    destination = Path(output_path)
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _plot_posterior_metric(
    axis,
    values: dict[str, np.ndarray],
    variants: list[str],
    metric: str,
    rng: np.random.Generator,
    bootstrap_samples: int,
) -> None:
    record_variants = values["posterior_variant"].astype(int)
    record_episodes = values["posterior_episode"].astype(int)
    record_times = values["posterior_time"]
    episode_count = values["seeds"].size
    for variant_index, variant in enumerate(variants):
        selected = record_variants == variant_index
        times = np.unique(record_times[selected])
        matrix = np.full((episode_count, times.size), np.nan, dtype=float)
        for time_index, time in enumerate(times):
            records = np.flatnonzero(selected & np.isclose(record_times, time))
            matrix[record_episodes[records], time_index] = values[metric][records]
        center, lower, upper = _bootstrap_band(
            matrix, rng, bootstrap_samples, allow_nan=True
        )
        color = COLORS.get(variant, f"C{variant_index}")
        axis.plot(times, center, color=color, linewidth=2.0)
        axis.fill_between(times, lower, upper, color=color, alpha=0.16)


def _bootstrap_band(
    samples: np.ndarray,
    rng: np.random.Generator,
    bootstrap_samples: int,
    *,
    allow_nan: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    matrix = np.asarray(samples, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] < 1:
        raise ValueError("samples must have shape (episodes, times)")
    reducer = np.nanmean if allow_nan else np.mean
    center = reducer(matrix, axis=0)
    indices = rng.integers(
        0, matrix.shape[0], size=(bootstrap_samples, matrix.shape[0])
    )
    resampled = reducer(matrix[indices], axis=1)
    lower, upper = np.nanpercentile(resampled, [2.5, 97.5], axis=0)
    return center, lower, upper


def main() -> None:
    arguments = parse_args()
    path = plot_ablation_evaluation(
        arguments.input,
        arguments.output,
        bootstrap_samples=arguments.bootstrap_samples,
    )
    print(f"ablation_plot={path}")


if __name__ == "__main__":
    main()
