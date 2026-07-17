"""Visualize the resolved views from the unified coupled configuration."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch

from src.coupled_config import load_coupled_configuration


def _box(axis, x, y, width, height, text, color) -> None:
    patch = FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.02",
        linewidth=1.5,
        edgecolor=color,
        facecolor=color,
        alpha=0.14,
    )
    axis.add_patch(patch)
    axis.text(
        x + width / 2,
        y + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=10,
        color="#172033",
    )


def generate_plot(config_path: Path, output_path: Path) -> Path:
    resolved = load_coupled_configuration(config_path)
    aerial = resolved.aerial
    ground = resolved.ground

    figure, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    flow = axes[0]
    flow.set_title("A. One YAML file resolves into two controller views")
    flow.set_xlim(0, 10)
    flow.set_ylim(0, 10)
    flow.axis("off")
    _box(flow, 2.6, 8.1, 4.8, 1.1, "configs/multifidelity.yaml", "#2563eb")
    _box(
        flow,
        0.4,
        6.0,
        3.8,
        1.2,
        "Shared\nsimulation · map · GP · output",
        "#7c3aed",
    )
    _box(flow, 5.8, 6.0, 1.8, 1.2, "aerial\noverrides", "#0284c7")
    _box(flow, 7.9, 6.0, 1.8, 1.2, "ground\noverrides", "#ea580c")
    _box(flow, 2.5, 3.9, 5.0, 1.1, "CoupledConfiguration loader", "#16a34a")
    _box(flow, 0.8, 1.4, 3.7, 1.3, "Aerial HEDACParams\nHEDAC + LOW sensor", "#0284c7")
    _box(flow, 5.5, 1.4, 3.7, 1.3, "Ground HEDACParams\nVoronoi/MPC + HIGH sensor", "#ea580c")
    for start, end in (
        ((5.0, 8.1), (2.3, 7.2)),
        ((5.0, 8.1), (6.7, 7.2)),
        ((5.0, 8.1), (8.8, 7.2)),
        ((2.3, 6.0), (4.1, 5.0)),
        ((6.7, 6.0), (5.4, 5.0)),
        ((8.8, 6.0), (6.1, 5.0)),
        ((4.2, 3.9), (2.7, 2.7)),
        ((5.8, 3.9), (7.3, 2.7)),
    ):
        flow.annotate(
            "",
            xy=end,
            xytext=start,
            arrowprops={"arrowstyle": "->", "color": "#64748b", "lw": 1.6},
        )

    table_axis = axes[1]
    table_axis.set_title("B. Values resolved by the real loader")
    table_axis.axis("off")
    rows = [
        ("Robot count", aerial.num_agents, ground.num_agents),
        (
            "Dynamics",
            aerial.get("agents.model_type"),
            ground.get("agents.model_type"),
        ),
        ("Controller", "HEDAC", "Voronoi + MPC"),
        ("Map size", str(aerial.map_config["size"]), str(ground.map_config["size"])),
        ("Resolution", aerial.resolution, ground.resolution),
        ("FOV depth", aerial.fov_depth, ground.fov_depth),
        (
            "Observation noise std",
            aerial.get("gpr.obs_noise_std"),
            ground.get("gpr.obs_noise_std"),
        ),
        ("Local query grid", "map cells", ground.local_grid_points),
    ]
    table = table_axis.table(
        cellText=rows,
        colLabels=("Resolved setting", "Aerial", "Ground"),
        cellLoc="center",
        colLoc="center",
        loc="center",
        colWidths=(0.42, 0.29, 0.29),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 2.0)
    for column in range(3):
        table[(0, column)].set_facecolor("#dbeafe")
        table[(0, column)].set_text_props(weight="bold")
    for row in range(1, len(rows) + 1):
        table[(row, 1)].set_facecolor("#e0f2fe")
        table[(row, 2)].set_facecolor("#ffedd5")
    table_axis.text(
        0.5,
        0.07,
        f"Shared: steps={aerial.num_steps}, dt={aerial.dt}, "
        f"seed={aerial.random_seed}, rho={aerial.get('multifidelity.rho')}",
        transform=table_axis.transAxes,
        ha="center",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#f1f5f9"},
    )

    figure.suptitle(
        "Unified multi-fidelity configuration: one source of truth, "
        "unchanged controller-specific parameters",
        fontsize=14,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=Path("configs/multifidelity.yaml")
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/assets/unified_config_milestone.png"),
    )
    arguments = parser.parse_args()
    print(f"saved={generate_plot(arguments.config, arguments.output)}")


if __name__ == "__main__":
    main()
