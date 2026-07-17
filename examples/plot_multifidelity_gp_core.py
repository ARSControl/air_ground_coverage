"""Visualize the deterministic synthetic test case for the multi-fidelity GP.

Run from the repository root with:

    python -m examples.plot_multifidelity_gp_core
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.core.multifidelity_gp import (  # noqa: E402
    MultiFidelityGaussianProcess,
    RBFKernel,
)


RHO = 0.8


def points_on_x_axis(x_values: np.ndarray) -> np.ndarray:
    """Convert one-dimensional x coordinates to repository-style positions."""
    return np.column_stack((x_values, np.zeros_like(x_values)))


def low_field(points: np.ndarray) -> np.ndarray:
    """Broad, smooth low-fidelity truth used by the numerical tests."""
    first = np.exp(
        -(
            (points[:, 0] + 1.1) ** 2
            + 0.55 * (points[:, 1] - 0.1) ** 2
        )
        / (2.0 * 1.45**2)
    )
    second = 0.5 * np.exp(
        -(
            (points[:, 0] - 1.6) ** 2
            + 0.8 * (points[:, 1] + 0.2) ** 2
        )
        / (2.0 * 1.0**2)
    )
    return first + second


def discrepancy_field(points: np.ndarray) -> np.ndarray:
    """Narrow correction visible only to the high-fidelity process."""
    return 0.7 * np.exp(
        -(
            (points[:, 0] - 0.55) ** 2
            + (points[:, 1] + 0.05) ** 2
        )
        / (2.0 * 0.2**2)
    )


def high_field(points: np.ndarray) -> np.ndarray:
    """Autoregressive high-fidelity truth."""
    return RHO * low_field(points) + discrepancy_field(points)


def make_gp() -> MultiFidelityGaussianProcess:
    """Build a GP with the exact kernel settings used by the tests."""
    return MultiFidelityGaussianProcess(
        rho=RHO,
        low_kernel=RBFKernel(length_scale=0.85, variance=1.5),
        discrepancy_kernel=RBFKernel(length_scale=0.2, variance=0.65),
        jitter=1.0e-9,
    )


def generate_plot(output_path: Path) -> Path:
    """Generate and save the four-panel deterministic diagnostic figure."""
    low_positions = points_on_x_axis(np.linspace(-3.5, 3.5, 29))
    high_positions = points_on_x_axis(np.array([0.38, 0.55, 0.72]))
    query = points_on_x_axis(np.linspace(-3.5, 3.5, 500))

    empty_positions = np.empty((0, 2), dtype=float)
    empty_values = np.empty(0, dtype=float)
    low_values = low_field(low_positions)
    high_values = high_field(high_positions)

    low_only = make_gp().fit(
        low_positions,
        low_values,
        1.0e-5,
        empty_positions,
        empty_values,
        0.0,
    )
    multifidelity = make_gp().fit(
        low_positions,
        low_values,
        1.0e-5,
        high_positions,
        high_values,
        1.0e-5,
    )
    low_prediction = low_only.predict_high(query)
    multifidelity_prediction = multifidelity.predict_high(query)

    x = query[:, 0]
    low_truth = low_field(query)
    discrepancy_truth = discrepancy_field(query)
    high_truth = high_field(query)
    low_error = np.abs(low_prediction.mean - high_truth)
    multifidelity_error = np.abs(multifidelity_prediction.mean - high_truth)

    colors = {
        "truth": "#172554",
        "low": "#2563eb",
        "high": "#dc2626",
        "multi": "#059669",
        "delta": "#d97706",
        "uncertainty": "#10b981",
    }
    figure, axes = plt.subplots(2, 2, figsize=(13, 9), sharex=True)
    figure.suptitle(
        "Autoregressive multi-fidelity GP: broad LOW structure + local HIGH correction",
        fontsize=15,
        fontweight="bold",
    )

    truth_axis = axes[0, 0]
    truth_axis.plot(
        x,
        RHO * low_truth,
        color=colors["low"],
        linewidth=2,
        label=r"$\rho f_L$ (broad component)",
    )
    truth_axis.plot(
        x,
        discrepancy_truth,
        color=colors["delta"],
        linewidth=2,
        label=r"$\delta$ (local discrepancy)",
    )
    truth_axis.plot(
        x,
        high_truth,
        color=colors["truth"],
        linewidth=2.5,
        label=r"$f_H = \rho f_L + \delta$",
    )
    truth_axis.set_title("A. Synthetic ground truth")
    truth_axis.set_ylabel("Field value")
    truth_axis.legend(loc="upper right", fontsize=9)

    estimate_axis = axes[0, 1]
    estimate_axis.plot(
        x,
        high_truth,
        color=colors["truth"],
        linewidth=2.5,
        label="HIGH ground truth",
    )
    estimate_axis.plot(
        x,
        low_prediction.mean,
        color=colors["low"],
        linestyle="--",
        linewidth=2,
        label="LOW-only posterior mean",
    )
    estimate_axis.plot(
        x,
        multifidelity_prediction.mean,
        color=colors["multi"],
        linewidth=2,
        label="Multi-fidelity posterior mean",
    )
    estimate_axis.scatter(
        low_positions[:, 0],
        RHO * low_values,
        facecolors="white",
        edgecolors=colors["low"],
        s=28,
        linewidths=1,
        label=r"LOW samples (shown as $\rho y_L$)",
        zorder=4,
    )
    estimate_axis.scatter(
        high_positions[:, 0],
        high_values,
        color=colors["high"],
        marker="x",
        s=55,
        linewidths=2,
        label="HIGH samples",
        zorder=5,
    )
    estimate_axis.set_title("B. Estimated latent HIGH field")
    estimate_axis.legend(loc="upper right", fontsize=8)

    error_axis = axes[1, 0]
    error_axis.semilogy(
        x,
        np.maximum(low_error, 1.0e-10),
        color=colors["low"],
        linestyle="--",
        linewidth=2,
        label="LOW-only absolute error",
    )
    error_axis.semilogy(
        x,
        np.maximum(multifidelity_error, 1.0e-10),
        color=colors["multi"],
        linewidth=2,
        label="Multi-fidelity absolute error",
    )
    error_axis.axvspan(0.3, 0.8, color=colors["delta"], alpha=0.12)
    error_axis.set_title("C. HIGH reconstruction error")
    error_axis.set_xlabel("x position (y = 0 slice)")
    error_axis.set_ylabel("Absolute error (log scale)")
    error_axis.legend(loc="upper right", fontsize=9)

    uncertainty_axis = axes[1, 1]
    uncertainty_axis.plot(
        x,
        np.sqrt(low_prediction.variance),
        color=colors["low"],
        linestyle="--",
        linewidth=2,
        label="LOW-only posterior std.",
    )
    uncertainty_axis.plot(
        x,
        np.sqrt(multifidelity_prediction.variance),
        color=colors["uncertainty"],
        linewidth=2,
        label="Multi-fidelity posterior std.",
    )
    uncertainty_axis.scatter(
        high_positions[:, 0],
        np.zeros(high_positions.shape[0]),
        color=colors["high"],
        marker="x",
        s=55,
        linewidths=2,
        label="HIGH sample locations",
        zorder=5,
    )
    uncertainty_axis.set_title("D. Uncertainty falls near HIGH observations")
    uncertainty_axis.set_xlabel("x position (y = 0 slice)")
    uncertainty_axis.set_ylabel("Posterior standard deviation")
    uncertainty_axis.legend(loc="upper right", fontsize=9)

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.set_xlim(x.min(), x.max())

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
        default=Path("docs/assets/multifidelity_gp_core_demo.png"),
        help="Output image path (default: %(default)s)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    result = generate_plot(arguments.output)
    print(result)
