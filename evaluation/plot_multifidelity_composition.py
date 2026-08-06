"""Plot reconstruction histories and summaries for the composition sweep."""

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

from evaluation.multifidelity_composition_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    load_archive,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_composition_evaluated.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_composition_metrics.png"),
    )
    parser.add_argument("--bootstrap-samples", type=int, default=1000)
    return parser.parse_args()


def plot_composition_evaluation(
    evaluated_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 1000,
) -> Path:
    """Render deterministic reconstruction histories and composition summaries."""
    if isinstance(bootstrap_samples, bool) or bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be a positive integer")
    metadata, values = load_archive(
        evaluated_path, expected_kind=EVALUATED_ARCHIVE_KIND
    )
    names = [str(value) for value in values["composition_names"].tolist()]
    colors = plt.get_cmap("viridis")(np.linspace(0.08, 0.92, max(len(names), 2)))
    rng = np.random.default_rng(20260804)
    figure, axes = plt.subplots(2, 3, figsize=(17.0, 8.2), constrained_layout=True)

    _plot_history(axes[0, 0], values, names, colors, "nrmse", rng, bootstrap_samples)
    axes[0, 0].set_title("A. Zero-clipped HIGH-field NRMSE (lower is better)")
    _plot_history(axes[0, 1], values, names, colors, "kl", rng, bootstrap_samples)
    axes[0, 1].set_title("B. Density KL divergence history (lower is better)")
    _plot_history(axes[0, 2], values, names, colors, "nlpd", rng, bootstrap_samples)
    axes[0, 2].set_title("C. Latent HIGH marginal NLPD (lower is better)")
    for axis in axes[0]:
        axis.set_xlabel("Mission time [s]")
        axis.grid(alpha=0.25)

    _plot_summary(axes[1, 0], values, "nrmse", "NRMSE", rng, bootstrap_samples)
    axes[1, 0].set_title("D. Final/time-average zero-clipped HIGH-field NRMSE")
    _plot_summary(axes[1, 1], values, "kl", "KL divergence", rng, bootstrap_samples)
    axes[1, 1].set_title("E. Final and time-averaged density KL")
    _plot_summary(axes[1, 2], values, "nlpd", "Marginal NLPD", rng, bootstrap_samples)
    axes[1, 2].set_title("F. Final and time-averaged latent HIGH NLPD")
    for axis in axes[1]:
        axis.set_xlabel("Aerial fraction $N_a/(N_a+N_g)$")
        axis.grid(alpha=0.25)
        axis.legend(frameon=False, fontsize=8)

    handles = [
        plt.Line2D([0], [0], color=colors[index], linewidth=2.2, label=name)
        for index, name in enumerate(names)
    ]
    figure.legend(
        handles=handles,
        loc="outside lower center",
        ncol=len(handles),
        frameon=False,
    )
    figure.suptitle(
        "Closed-loop reconstruction across fixed-size team compositions\n"
        f"scenario={metadata.get('scenario', 'default')}; "
        f"kernels={_hyperparameter_policy(metadata)}; "
        f"N={metadata['total_robots']}; lines/markers: episode mean; "
        "bands/error bars: bootstrap 95% interval",
        fontsize=14,
    )
    destination = Path(output_path)
    if not destination.name:
        raise ValueError("output_path must name a file")
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _hyperparameter_policy(metadata: dict) -> str:
    settings = metadata.get("hyperparameter_optimization", {"enabled": False})
    if isinstance(settings, dict) and settings.get("enabled", False):
        return "online optimized"
    return "fixed"


def _plot_history(
    axis,
    values: dict[str, np.ndarray],
    names: list[str],
    colors: np.ndarray,
    metric: str,
    rng: np.random.Generator,
    bootstrap_samples: int,
) -> None:
    record_compositions = values["posterior_composition"].astype(int)
    record_episodes = values["posterior_episode"].astype(int)
    record_times = values["posterior_time"]
    episode_count = values["seeds"].size
    for composition, name in enumerate(names):
        selected = record_compositions == composition
        times = np.unique(record_times[selected])
        matrix = np.full((episode_count, times.size), np.nan, dtype=float)
        for time_index, timestamp in enumerate(times):
            records = np.flatnonzero(selected & np.isclose(record_times, timestamp))
            matrix[record_episodes[records], time_index] = values[metric][records]
        center, lower, upper = _bootstrap_mean(
            matrix, rng, bootstrap_samples, allow_nan=True
        )
        axis.plot(times, center, color=colors[composition], linewidth=2.0, label=name)
        axis.fill_between(times, lower, upper, color=colors[composition], alpha=0.14)


def _plot_summary(
    axis,
    values: dict[str, np.ndarray],
    metric: str,
    ylabel: str,
    rng: np.random.Generator,
    bootstrap_samples: int,
) -> None:
    total = values["aerial_counts"] + values["ground_counts"]
    fractions = values["aerial_counts"] / total
    order = np.argsort(fractions)
    offsets = (-0.012, 0.012)
    for offset, (prefix, label, marker) in zip(
        offsets,
        (
            ("final", "Final", "o"),
            ("time_average", "Time average", "s"),
        ),
        strict=True,
    ):
        samples = values[f"{prefix}_{metric}"][order]
        center, lower, upper = _bootstrap_mean(samples.T, rng, bootstrap_samples)
        x_values = fractions[order] + offset
        axis.errorbar(
            x_values,
            center,
            yerr=np.vstack((center - lower, upper - center)),
            color="#303030" if prefix == "final" else "#C44E52",
            marker=marker,
            linewidth=1.5,
            capsize=3,
            label=label,
        )
    axis.set_ylabel(ylabel)
    axis.set_xticks(fractions[order])
    axis.set_xlim(-0.06, 1.06)


def _bootstrap_mean(
    samples: np.ndarray,
    rng: np.random.Generator,
    bootstrap_samples: int,
    *,
    allow_nan: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    matrix = np.asarray(samples, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] < 1:
        raise ValueError("samples must have shape (episodes, values)")
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
    path = plot_composition_evaluation(
        arguments.input,
        arguments.output,
        bootstrap_samples=arguments.bootstrap_samples,
    )
    print(f"composition_plot={path}")


if __name__ == "__main__":
    main()
