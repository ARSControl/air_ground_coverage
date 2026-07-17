"""Plot deterministic fixed-versus-fitted multi-fidelity GP behavior."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

from src.core.multifidelity_gp import (
    KernelHyperparameterBounds,
    MultiFidelityGaussianProcess,
    RBFKernel,
)


def _points(x_values: np.ndarray) -> np.ndarray:
    return np.column_stack((x_values, np.zeros_like(x_values)))


def generate_plot(output_path: Path) -> Path:
    rho = 0.8
    low_x = np.linspace(-4.0, 4.0, 17)
    high_x = np.linspace(-3.5, 3.5, 11)
    query_x = np.linspace(-4.0, 4.0, 301)
    low_values = np.exp(-0.5 * (low_x / 1.8) ** 2)
    high_values = rho * np.exp(-0.5 * (high_x / 1.8) ** 2) + 0.35 * np.exp(
        -0.5 * ((high_x - 1.0) / 0.7) ** 2
    )
    truth = rho * np.exp(-0.5 * (query_x / 1.8) ** 2) + 0.35 * np.exp(
        -0.5 * ((query_x - 1.0) / 0.7) ** 2
    )
    gp = MultiFidelityGaussianProcess(
        rho=rho,
        low_kernel=RBFKernel(0.35, 0.5),
        discrepancy_kernel=RBFKernel(0.25, 0.5),
        jitter=1.0e-8,
    )
    initial_parameters = gp.kernel_hyperparameters
    gp.fit(
        _points(low_x),
        low_values,
        1.0e-4,
        _points(high_x),
        high_values,
        1.0e-4,
    )
    fixed = gp.predict_high(_points(query_x))
    fit_result = gp.fit_hyperparameters(
        _points(low_x),
        low_values,
        1.0e-4,
        _points(high_x),
        high_values,
        1.0e-4,
        bounds=KernelHyperparameterBounds(
            low_length_scale=(0.2, 5.0),
            low_variance=(0.05, 3.0),
            discrepancy_length_scale=(0.2, 3.0),
            discrepancy_variance=(0.01, 2.0),
        ),
        num_restarts=1,
        max_iterations=100,
    )
    fitted_parameters = gp.kernel_hyperparameters
    gp.fit(
        _points(low_x),
        low_values,
        1.0e-4,
        _points(high_x),
        high_values,
        1.0e-4,
    )
    fitted = gp.predict_high(_points(query_x))
    fixed_rmse = float(np.sqrt(np.mean((fixed.mean - truth) ** 2)))
    fitted_rmse = float(np.sqrt(np.mean((fitted.mean - truth) ** 2)))

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(query_x, truth, color="black", lw=2.5, label="HIGH truth")
    axes[0, 0].scatter(low_x, rho * low_values, color="#2563eb", label="rho × LOW")
    axes[0, 0].scatter(high_x, high_values, color="#f97316", marker="X", label="HIGH")
    axes[0, 0].set_title("A. Deterministic training case")
    axes[0, 0].legend(fontsize=8)

    for axis, prediction, title, color in (
        (axes[0, 1], fixed, "B. Fixed short length scales", "#dc2626"),
        (axes[1, 0], fitted, "C. Bounded marginal-likelihood fit", "#16a34a"),
    ):
        standard_deviation = np.sqrt(prediction.variance)
        axis.plot(query_x, truth, color="black", lw=2, label="HIGH truth")
        axis.plot(query_x, prediction.mean, color=color, lw=2, label="GP mean")
        axis.fill_between(
            query_x,
            prediction.mean - 2.0 * standard_deviation,
            prediction.mean + 2.0 * standard_deviation,
            color=color,
            alpha=0.18,
            label="±2 std",
        )
        axis.set_title(title)
        axis.legend(fontsize=8)

    metric_axis = axes[1, 1]
    labels = ("fixed", "fitted")
    rmse_values = (fixed_rmse, fitted_rmse)
    x_positions = np.arange(len(labels))
    metric_axis.bar(
        x_positions,
        rmse_values,
        width=0.55,
        color=("#dc2626", "#16a34a"),
    )
    metric_axis.set_xticks(x_positions, labels)
    metric_axis.set_yscale("log")
    metric_axis.set_ylim(0.0025, 0.08)
    metric_axis.set_ylabel("high-fidelity prediction RMSE (log scale)")
    metric_axis.set_title("D. Fit diagnostics")
    for x_position, value in zip(x_positions, rmse_values, strict=True):
        metric_axis.text(
            x_position,
            value * 1.08,
            f"{value:.4f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    metric_axis.text(
        0.47,
        0.96,
        "Marginal-likelihood objective (lower is better)\n"
        f"{fit_result.initial_negative_log_likelihood:.2f} → "
        f"{fit_result.final_negative_log_likelihood:.2f}\n\n"
        "Kernel parameters: initial → fitted\n"
        f"LOW length: {initial_parameters[0]:.3f} → {fitted_parameters[0]:.3f}\n"
        f"LOW variance: {initial_parameters[1]:.3f} → {fitted_parameters[1]:.3f}\n"
        "Discrepancy length: "
        f"{initial_parameters[2]:.3f} → {fitted_parameters[2]:.3f}\n"
        "Discrepancy variance: "
        f"{initial_parameters[3]:.3f} → {fitted_parameters[3]:.3f}",
        transform=metric_axis.transAxes,
        va="top",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cbd5e1"},
    )

    for axis in axes.flat:
        axis.grid(True, alpha=0.2)
    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.set_xlabel("x")
    axes[0, 0].set_ylabel("field value")
    axes[0, 1].set_ylabel("field value")
    axes[1, 0].set_ylabel("field value")
    figure.suptitle(
        "Central multi-fidelity GP hyperparameter fitting\n"
        "fixed rho and sensor noise; four RBF parameters fitted in log-space",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    print(f"fixed_rmse={fixed_rmse:.12f}")
    print(f"fitted_rmse={fitted_rmse:.12f}")
    print(
        "negative_log_likelihood="
        f"{fit_result.initial_negative_log_likelihood:.12f}->"
        f"{fit_result.final_negative_log_likelihood:.12f}"
    )
    print(f"fitted_parameters={fitted_parameters}")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/hyperparameter_fitting_milestone.png"),
    )
    arguments = parser.parse_args()
    print(f"saved={generate_plot(arguments.output)}")


if __name__ == "__main__":
    main()
