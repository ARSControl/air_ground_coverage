"""Generate the deterministic visual summary for the async-estimator milestone.

Run from the repository root with:

    python -m examples.plot_async_estimator_milestone
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.core.multifidelity_estimator import (  # noqa: E402
    CentralAsynchronousEstimator,
    EstimatorSettings,
    PosteriorSnapshot,
    UpdateStatus,
)
from src.core.multifidelity_gp import (  # noqa: E402
    MultiFidelityGaussianProcess,
    RBFKernel,
)
from src.core.observations import (  # noqa: E402
    Fidelity,
    Observation,
    RetentionConfig,
)


RHO = 0.8


@dataclass(frozen=True)
class Arrival:
    arrival_time: float
    observation: Observation


def low_truth(x_position: float | np.ndarray) -> float | np.ndarray:
    return np.exp(-0.5 * (np.asarray(x_position) / 1.2) ** 2)


def high_truth(x_position: float | np.ndarray) -> float | np.ndarray:
    x_values = np.asarray(x_position)
    discrepancy = 0.7 * np.exp(-0.5 * ((x_values - 0.5) / 0.2) ** 2)
    return RHO * low_truth(x_values) + discrepancy


def make_observation(
    timestamp: float,
    x_position: float,
    fidelity: Fidelity,
    robot_id: str,
) -> Observation:
    field_value = (
        low_truth(x_position)
        if fidelity is Fidelity.LOW
        else high_truth(x_position)
    )
    return Observation(
        timestamp=timestamp,
        robot_id=robot_id,
        position=(x_position, 0.0),
        value=float(field_value),
        fidelity=fidelity,
        noise_variance=1.0e-4,
    )


def make_estimator() -> CentralAsynchronousEstimator:
    x_values = np.linspace(-3.0, 3.0, 121)
    query_points = np.column_stack((x_values, np.zeros_like(x_values)))
    weights = np.full(x_values.size, x_values[1] - x_values[0])
    weights[[0, -1]] *= 0.5
    settings = EstimatorSettings(
        gp=MultiFidelityGaussianProcess(
            rho=RHO,
            low_kernel=RBFKernel(length_scale=0.9, variance=1.4),
            discrepancy_kernel=RBFKernel(length_scale=0.25, variance=0.6),
            jitter=1.0e-9,
        ),
        low_retention=RetentionConfig(max_samples=20, min_separation=0.0),
        high_retention=RetentionConfig(max_samples=20, min_separation=0.0),
        query_points=query_points,
        query_shape=(x_values.size,),
        integration_weights=weights,
        normalization_tolerance=1.0e-9,
    )
    return CentralAsynchronousEstimator(settings)


def run_event_history() -> tuple[
    list[Arrival],
    list[tuple[float, UpdateStatus, int, int, int]],
    PosteriorSnapshot,
    PosteriorSnapshot,
]:
    arrivals = [
        Arrival(0.00, make_observation(0.00, -2.5, Fidelity.LOW, "aerial-0")),
        Arrival(0.20, make_observation(0.20, -1.5, Fidelity.LOW, "aerial-0")),
        Arrival(0.40, make_observation(0.40, -0.5, Fidelity.LOW, "aerial-0")),
        Arrival(0.60, make_observation(0.60, 0.5, Fidelity.LOW, "aerial-0")),
        Arrival(0.80, make_observation(0.80, 1.5, Fidelity.LOW, "aerial-0")),
        Arrival(0.95, make_observation(0.95, 2.5, Fidelity.LOW, "aerial-0")),
        # Delayed HIGH data: collected at 0.70, delivered after the first update.
        Arrival(1.15, make_observation(0.70, 0.35, Fidelity.HIGH, "ground-0")),
        Arrival(1.30, make_observation(1.20, 0.50, Fidelity.HIGH, "ground-1")),
        # Out-of-order timestamp: collected before the preceding arrival.
        Arrival(1.45, make_observation(1.00, 0.65, Fidelity.HIGH, "ground-2")),
        Arrival(1.60, make_observation(1.60, 2.9, Fidelity.LOW, "aerial-0")),
    ]
    estimator = make_estimator()
    updates: list[tuple[float, UpdateStatus, int, int, int]] = []

    for arrival in arrivals[:6]:
        estimator.submit(arrival.observation)
    first_report = estimator.update(1.0)
    first_snapshot = estimator.latest_posterior
    assert first_snapshot is not None
    updates.append(
        (
            1.0,
            first_report.status,
            estimator.version,
            estimator.low_sample_count,
            estimator.high_sample_count,
        )
    )

    for arrival in arrivals[6:]:
        estimator.submit(arrival.observation)
    second_report = estimator.update(2.0)
    second_snapshot = estimator.latest_posterior
    assert second_snapshot is not None
    updates.append(
        (
            2.0,
            second_report.status,
            estimator.version,
            estimator.low_sample_count,
            estimator.high_sample_count,
        )
    )

    no_data_report = estimator.update(2.5)
    updates.append(
        (
            2.5,
            no_data_report.status,
            estimator.version,
            estimator.low_sample_count,
            estimator.high_sample_count,
        )
    )
    return arrivals, updates, first_snapshot, second_snapshot


def generate_plot(output_path: Path) -> Path:
    arrivals, updates, low_snapshot, joint_snapshot = run_event_history()
    colors = {
        Fidelity.LOW: "#2563eb",
        Fidelity.HIGH: "#dc2626",
        "joint": "#059669",
        "truth": "#172554",
        "uncertainty": "#d97706",
    }
    figure, axes = plt.subplots(2, 2, figsize=(13, 9))
    figure.suptitle(
        "Central asynchronous estimator: buffered arrivals and explicit updates",
        fontsize=15,
        fontweight="bold",
    )

    arrival_axis = axes[0, 0]
    for fidelity in Fidelity:
        selected = [
            arrival for arrival in arrivals if arrival.observation.fidelity is fidelity
        ]
        arrival_axis.scatter(
            [arrival.arrival_time for arrival in selected],
            [arrival.observation.timestamp for arrival in selected],
            color=colors[fidelity],
            s=55,
            label=f"{fidelity.name} observation",
            zorder=3,
        )
    arrival_axis.plot([0.0, 2.0], [0.0, 2.0], color="#64748b", linestyle=":")
    for update_time in (1.0, 2.0, 2.5):
        arrival_axis.axvline(update_time, color="#111827", alpha=0.2, linewidth=1)
    arrival_axis.annotate(
        "delayed / out-of-order HIGH data",
        xy=(1.15, 0.7),
        xytext=(0.15, 1.35),
        arrowprops={"arrowstyle": "->", "color": colors[Fidelity.HIGH]},
        fontsize=9,
    )
    arrival_axis.set_title("A. Collection time is independent of arrival time")
    arrival_axis.set_xlabel("Arrival time")
    arrival_axis.set_ylabel("Observation timestamp")
    arrival_axis.legend(loc="lower right", fontsize=9)

    update_axis = axes[0, 1]
    update_times = np.asarray([record[0] for record in updates])
    versions = np.asarray([record[2] for record in updates])
    low_counts = np.asarray([record[3] for record in updates])
    high_counts = np.asarray([record[4] for record in updates])
    update_axis.step(
        update_times,
        versions,
        where="post",
        color="#111827",
        linewidth=2.5,
        label="Published version",
    )
    update_axis.plot(
        update_times,
        low_counts,
        color=colors[Fidelity.LOW],
        marker="o",
        linewidth=2,
        label="Retained LOW count",
    )
    update_axis.plot(
        update_times,
        high_counts,
        color=colors[Fidelity.HIGH],
        marker="x",
        markersize=8,
        linewidth=2,
        label="Retained HIGH count",
    )
    for update_time, status, version, _, _ in updates:
        update_axis.annotate(
            status.name,
            (update_time, version),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            fontsize=8,
        )
    update_axis.set_title("B. Only explicit updates publish versions")
    update_axis.set_xlabel("Scheduler update time")
    update_axis.set_ylabel("Version / retained samples")
    update_axis.set_xticks(update_times)
    update_axis.legend(loc="upper left", fontsize=9)

    x_values = joint_snapshot.query_points[:, 0]
    truth = np.asarray(high_truth(x_values))
    posterior_axis = axes[1, 0]
    posterior_axis.plot(
        x_values,
        truth,
        color=colors["truth"],
        linewidth=2.5,
        label="HIGH ground truth",
    )
    posterior_axis.plot(
        x_values,
        low_snapshot.high_mean,
        color=colors[Fidelity.LOW],
        linestyle="--",
        linewidth=2,
        label="Version 1: LOW only",
    )
    posterior_axis.plot(
        x_values,
        joint_snapshot.high_mean,
        color=colors["joint"],
        linewidth=2,
        label="Version 2: delayed HIGH included",
    )
    high_arrivals = [
        arrival for arrival in arrivals if arrival.observation.fidelity is Fidelity.HIGH
    ]
    posterior_axis.scatter(
        [arrival.observation.position[0] for arrival in high_arrivals],
        [arrival.observation.value for arrival in high_arrivals],
        color=colors[Fidelity.HIGH],
        marker="x",
        s=55,
        linewidths=2,
        label="HIGH samples",
        zorder=4,
    )
    posterior_axis.set_title("C. Later HIGH data creates the local correction")
    posterior_axis.set_xlabel("x position (y = 0 slice)")
    posterior_axis.set_ylabel("Latent HIGH field")
    posterior_axis.legend(loc="upper right", fontsize=9)

    density_axis = axes[1, 1]
    density_axis.plot(
        x_values,
        low_snapshot.density,
        color=colors[Fidelity.LOW],
        linestyle="--",
        linewidth=2,
        label="Version 1 density",
    )
    density_axis.plot(
        x_values,
        joint_snapshot.density,
        color=colors["joint"],
        linewidth=2,
        label="Version 2 density",
    )
    uncertainty_axis = density_axis.twinx()
    uncertainty_axis.plot(
        x_values,
        np.sqrt(low_snapshot.high_variance),
        color=colors["uncertainty"],
        linestyle=":",
        linewidth=1.7,
        alpha=0.8,
        label="Version 1 posterior std.",
    )
    uncertainty_axis.plot(
        x_values,
        np.sqrt(joint_snapshot.high_variance),
        color=colors["uncertainty"],
        linestyle="-.",
        linewidth=1.7,
        alpha=0.8,
        label="Version 2 posterior std.",
    )
    density_axis.set_title("D. Cached density and uncertainty change atomically")
    density_axis.set_xlabel("x position (y = 0 slice)")
    density_axis.set_ylabel("Normalized density")
    uncertainty_axis.set_ylabel("Posterior standard deviation")
    handles, labels = density_axis.get_legend_handles_labels()
    other_handles, other_labels = uncertainty_axis.get_legend_handles_labels()
    density_axis.legend(
        handles + other_handles,
        labels + other_labels,
        loc="upper left",
        fontsize=8,
    )

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/async_estimator_milestone.png"),
        help="Output image path (default: %(default)s)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    print(generate_plot(arguments.output))
