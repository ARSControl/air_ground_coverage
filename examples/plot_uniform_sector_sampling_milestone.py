"""Reproduce the historical fixed-ray versus uniform-sector comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
from types import SimpleNamespace

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

from src.core.observations import Fidelity
from src.models.sensors import SimulatedScalarFieldSensor


def _uniform_sector_positions() -> np.ndarray:
    center = np.array([25.0, 25.0])
    team = SimpleNamespace(
        agents=[SimpleNamespace(id=0, position=center, theta=0.0)]
    )
    sensor = SimulatedScalarFieldSensor(
        Fidelity.HIGH,
        sample_count=10,
        sample_range=15.0,
        fov_degrees=360.0,
        noise_variance=0.0,
        rng=np.random.default_rng(2026),
    )
    field = np.ones((51, 51), dtype=float)
    observations = []
    for event_index in range(30):
        observations.extend(sensor.collect(team, field, float(event_index)))
    return np.asarray([item.position for item in observations], dtype=float)


def _historical_fixed_ray_positions() -> np.ndarray:
    """Reconstruct the removed fixed-ray baseline without a sensor mode."""
    center = np.array([25.0, 25.0])
    sample_range = 15.0
    sample_count = 10
    angles = np.linspace(0.0, 2.0 * np.pi, sample_count, endpoint=False)
    rng = np.random.default_rng(2026)
    positions = []
    for _ in range(30):
        radii = rng.uniform(0.0, sample_range, sample_count)
        positions.extend(
            center + radius * np.array([np.cos(angle), np.sin(angle)])
            for angle, radius in zip(angles, radii)
        )
    return np.asarray(positions, dtype=float)


def _scatter_panel(
    axis,
    positions: np.ndarray,
    title: str,
    center: np.ndarray,
    sample_range: float,
) -> None:
    event_index = np.repeat(np.arange(30), 10)
    axis.scatter(
        positions[:, 0],
        positions[:, 1],
        c=event_index,
        cmap="viridis",
        s=13,
        alpha=0.72,
        edgecolors="none",
    )
    axis.scatter(*center, marker="X", s=90, color="#dc2626", label="robot")
    axis.add_patch(
        plt.Circle(center, sample_range, fill=False, ls="--", color="black", lw=1)
    )
    axis.set_title(title)
    axis.set_xlim(center[0] - sample_range - 1, center[0] + sample_range + 1)
    axis.set_ylim(center[1] - sample_range - 1, center[1] + sample_range + 1)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x")
    axis.set_ylabel("y")
    axis.grid(True, alpha=0.2)
    axis.legend(loc="upper right", fontsize=8)


def generate_plot(output_path: Path) -> Path:
    center = np.array([25.0, 25.0])
    sample_range = 15.0
    rays = _historical_fixed_ray_positions()
    sector = _uniform_sector_positions()
    ray_radius = np.linalg.norm(rays - center, axis=1) / sample_range
    sector_radius = np.linalg.norm(sector - center, axis=1) / sample_range

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    _scatter_panel(
        axes[0],
        rays,
        "A. Legacy: fixed bearings + uniform radius",
        center,
        sample_range,
    )
    _scatter_panel(
        axes[1],
        sector,
        "B. New: random bearing + area-uniform radius",
        center,
        sample_range,
    )

    probability = np.arange(1, ray_radius.size + 1) / ray_radius.size
    axes[2].plot(
        np.sort(ray_radius), probability, lw=2, label="fixed rays (observed)"
    )
    axes[2].plot(
        np.sort(sector_radius),
        probability,
        lw=2,
        label="uniform sector (observed)",
    )
    normalized_radius = np.linspace(0.0, 1.0, 200)
    axes[2].plot(
        normalized_radius,
        normalized_radius**2,
        "k--",
        label="ideal uniform-area CDF: r²",
    )
    axes[2].set_title("C. Normalized radial distribution")
    axes[2].set_xlabel("radius / sensor range")
    axes[2].set_ylabel("empirical cumulative probability")
    axes[2].set_xlim(0.0, 1.0)
    axes[2].set_ylim(0.0, 1.0)
    axes[2].grid(True, alpha=0.2)
    axes[2].legend(fontsize=8)
    axes[2].text(
        0.03,
        0.97,
        "mean squared normalized radius\n"
        f"fixed rays: {np.mean(ray_radius**2):.3f}\n"
        f"uniform sector: {np.mean(sector_radius**2):.3f}\n"
        "ideal uniform area: 0.500",
        transform=axes[2].transAxes,
        va="top",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cbd5e1"},
    )
    figure.suptitle(
        "Ground HIGH sensor sampling — 30 events, 10 samples/event, fixed seed",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    print(f"fixed_ray_mean_squared_radius={np.mean(ray_radius**2):.12f}")
    print(f"uniform_sector_mean_squared_radius={np.mean(sector_radius**2):.12f}")
    print(f"saved={output_path}")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/uniform_sector_sampling_milestone.png"),
    )
    arguments = parser.parse_args()
    generate_plot(arguments.output)


if __name__ == "__main__":
    main()
