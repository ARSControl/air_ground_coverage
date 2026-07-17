"""Visualize and validate the sensor's unconditional uniform-sector law."""

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


SAMPLE_COUNT = 4096
CENTER = np.array([50.0, 50.0])
HEADING = 0.35
FOV_DEGREES = 120.0
SAMPLE_RANGE = 20.0


def _sample_positions() -> np.ndarray:
    team = SimpleNamespace(
        agents=[SimpleNamespace(id=0, position=CENTER, theta=HEADING)]
    )
    sensor = SimulatedScalarFieldSensor(
        Fidelity.HIGH,
        sample_count=SAMPLE_COUNT,
        sample_range=SAMPLE_RANGE,
        fov_degrees=FOV_DEGREES,
        noise_variance=0.0,
        rng=np.random.default_rng(2026),
    )
    observations = sensor.collect(team, np.ones((101, 101)), 0.0)
    return np.asarray([item.position for item in observations], dtype=float)


def generate_plot(output_path: Path) -> Path:
    positions = _sample_positions()
    offsets = positions - CENTER
    normalized_radius = np.linalg.norm(offsets, axis=1) / SAMPLE_RANGE
    half_fov = 0.5 * np.deg2rad(FOV_DEGREES)
    relative_angle = np.arctan2(offsets[:, 1], offsets[:, 0]) - HEADING
    normalized_angle = relative_angle / half_fov

    radial_probability = np.arange(1, SAMPLE_COUNT + 1) / SAMPLE_COUNT
    angular_probability = radial_probability.copy()
    ideal_axis = np.linspace(0.0, 1.0, 300)
    ideal_angle = np.linspace(-1.0, 1.0, 300)
    radial_ks = float(
        np.max(np.abs(radial_probability - np.sort(normalized_radius) ** 2))
    )
    angular_ks = float(
        np.max(
            np.abs(
                angular_probability
                - 0.5 * (np.sort(normalized_angle) + 1.0)
            )
        )
    )

    figure, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)

    axes[0].scatter(
        positions[:, 0],
        positions[:, 1],
        c=normalized_radius,
        cmap="viridis",
        s=7,
        alpha=0.55,
        edgecolors="none",
    )
    axes[0].scatter(*CENTER, marker="X", s=100, color="#dc2626", label="robot")
    edge_angles = HEADING + np.array([-half_fov, half_fov])
    for edge_angle in edge_angles:
        endpoint = CENTER + SAMPLE_RANGE * np.array(
            [np.cos(edge_angle), np.sin(edge_angle)]
        )
        axes[0].plot(
            [CENTER[0], endpoint[0]],
            [CENTER[1], endpoint[1]],
            "k--",
            lw=1,
        )
    arc_angles = np.linspace(edge_angles[0], edge_angles[1], 200)
    axes[0].plot(
        CENTER[0] + SAMPLE_RANGE * np.cos(arc_angles),
        CENTER[1] + SAMPLE_RANGE * np.sin(arc_angles),
        "k--",
        lw=1,
    )
    axes[0].set_title("A. Samples fill the FOV sector")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set_xlabel("x")
    axes[0].set_ylabel("y")
    axes[0].grid(True, alpha=0.2)
    axes[0].legend(loc="upper left", fontsize=8)

    axes[1].plot(
        np.sort(normalized_radius),
        radial_probability,
        lw=2,
        label="implemented sensor",
    )
    axes[1].plot(ideal_axis, ideal_axis**2, "k--", label="ideal $F(r)=r^2$")
    axes[1].set_title("B. Area-uniform radial CDF")
    axes[1].set_xlabel("radius / sensor range")
    axes[1].set_ylabel("cumulative probability")
    axes[1].set_xlim(0.0, 1.0)
    axes[1].set_ylim(0.0, 1.0)
    axes[1].grid(True, alpha=0.2)
    axes[1].legend(fontsize=8)
    axes[1].text(
        0.04,
        0.96,
        f"mean $r^2$ = {np.mean(normalized_radius**2):.4f}\n"
        f"ideal = 0.5000\nKS distance = {radial_ks:.4f}",
        transform=axes[1].transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cbd5e1"},
    )

    axes[2].plot(
        np.sort(normalized_angle),
        angular_probability,
        lw=2,
        label="implemented sensor",
    )
    axes[2].plot(
        ideal_angle,
        0.5 * (ideal_angle + 1.0),
        "k--",
        label="ideal uniform angle",
    )
    axes[2].set_title("C. Uniform angular CDF")
    axes[2].set_xlabel("relative angle / half FOV")
    axes[2].set_ylabel("cumulative probability")
    axes[2].set_xlim(-1.0, 1.0)
    axes[2].set_ylim(0.0, 1.0)
    axes[2].grid(True, alpha=0.2)
    axes[2].legend(fontsize=8)
    axes[2].text(
        0.04,
        0.96,
        f"mean normalized angle = {np.mean(normalized_angle):.4f}\n"
        f"ideal = 0.0000\nKS distance = {angular_ks:.4f}",
        transform=axes[2].transAxes,
        va="top",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cbd5e1"},
    )

    figure.suptitle(
        "Uniform-sector-only sensor — 4096 samples, 120° FOV, fixed seed",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)

    print(f"mean_normalized_squared_radius={np.mean(normalized_radius**2):.12f}")
    print(f"mean_normalized_angle={np.mean(normalized_angle):.12f}")
    print(f"radial_ks_distance={radial_ks:.12f}")
    print(f"angular_ks_distance={angular_ks:.12f}")
    print(f"saved={output_path}")
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/uniform_sector_only_milestone.png"),
    )
    arguments = parser.parse_args()
    generate_plot(arguments.output)


if __name__ == "__main__":
    main()
