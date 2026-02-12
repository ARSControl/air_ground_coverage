"""
Visualization utilities for GP-based HEDAC debugging.
"""

from typing import Optional, Tuple
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize


def _plot_obstacles(ax: plt.Axes, map_array: Optional[np.ndarray], alpha: float = 0.3):
    if map_array is None:
        return
    if np.any(map_array == 1):
        ax.pcolormesh(
            np.where(map_array == 0, np.nan, map_array), cmap="gray", alpha=alpha
        )


def _plot_trajectory(
    ax: plt.Axes, trajectory: np.ndarray, cmap: str = "viridis"
) -> Optional[LineCollection]:
    if trajectory is None or len(trajectory) < 2:
        return None
    points = trajectory.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = Normalize(vmin=0, vmax=max(len(trajectory) - 1, 1))
    lc = LineCollection(segments, cmap=cmap, norm=norm)
    lc.set_array(np.arange(len(trajectory) - 1))
    lc.set_linewidth(2.0)
    ax.add_collection(lc)
    return lc


def visualize_gp_debug(
    *,
    step_num: int,
    map_array: Optional[np.ndarray],
    true_goal_density: Optional[np.ndarray],
    combined_goal_density: Optional[np.ndarray],
    coverage_density: Optional[np.ndarray],
    ergodic_metrics: Optional[np.ndarray],
    trajectory: Optional[np.ndarray],
    current_position: Optional[np.ndarray],
    observations_all: Optional[np.ndarray],
    observations_filtered: Optional[np.ndarray],
    save_path: Optional[str] = None,
    show: bool = True,
    cmap_goal: str = "RdPu",
    cmap_combined: str = "viridis",
    cmap_coverage: str = "hot",
) -> Tuple[plt.Figure, np.ndarray]:
    """
    Create a 4-panel GP debugging visualization.

    Panel 1: Robot trajectory over true goal density with observations
    Panel 2: Combined goal density (mean + std)
    Panel 3: Coverage density
    Panel 4: Ergodic metric over time
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Panel 1: True goal density + trajectory + observations
    ax = axes[0, 0]
    if true_goal_density is not None:
        im_goal = ax.imshow(true_goal_density, cmap=cmap_goal, origin="lower")
        _plot_obstacles(ax, map_array)
        fig.colorbar(im_goal, ax=ax, fraction=0.046, pad=0.04)

    if trajectory is not None and len(trajectory) > 1:
        _plot_trajectory(ax, trajectory)

    if observations_all is not None and observations_all.size > 0:
        ax.scatter(
            observations_all[:, 0],
            observations_all[:, 1],
            c="lightgray",
            s=15,
            alpha=0.3,
            label="all obs",
        )

    if observations_filtered is not None and observations_filtered.size > 0:
        ax.scatter(
            observations_filtered[:, 0],
            observations_filtered[:, 1],
            c="red",
            s=45,
            alpha=0.8,
            label="dataset",
        )

    if current_position is not None:
        ax.scatter(
            current_position[0],
            current_position[1],
            c="cyan",
            s=90,
            marker="*",
            edgecolors="black",
        )

    total_count = 0 if observations_all is None else int(observations_all.shape[0])
    filtered_count = (
        0 if observations_filtered is None else int(observations_filtered.shape[0])
    )
    ax.set_title(
        f"Robot Trajectory & Observations\nDataset: {filtered_count}/{total_count}"
    )
    ax.set_aspect("equal")
    ax.legend(loc="upper right")

    # Panel 2: Combined goal density
    ax = axes[0, 1]
    if combined_goal_density is not None:
        im_combo = ax.imshow(combined_goal_density, cmap=cmap_combined, origin="lower")
        _plot_obstacles(ax, map_array)
        fig.colorbar(im_combo, ax=ax, fraction=0.046, pad=0.04)
    if current_position is not None:
        ax.scatter(
            current_position[0],
            current_position[1],
            c="cyan",
            s=70,
            marker="o",
            edgecolors="black",
        )
    ax.set_title("Combined Goal Density")
    ax.set_aspect("equal")

    # Panel 3: Coverage density
    ax = axes[1, 0]
    if coverage_density is not None:
        coverage_norm = coverage_density / (np.sum(coverage_density) + 1e-10)
        im_cov = ax.imshow(coverage_norm, cmap=cmap_coverage, origin="lower")
        _plot_obstacles(ax, map_array)
        fig.colorbar(im_cov, ax=ax, fraction=0.046, pad=0.04)
    ax.set_title("Coverage Density")
    ax.set_aspect("equal")

    # Panel 4: Ergodic metric
    ax = axes[1, 1]
    if ergodic_metrics is not None and len(ergodic_metrics) > 0:
        ax.plot(ergodic_metrics, "b-", linewidth=2)
        ax.axvline(x=step_num, color="r", linestyle="--", alpha=0.5)
        ax.set_title(f"Ergodic Metric: {ergodic_metrics[-1]:.6f}")
    else:
        ax.set_title("Ergodic Metric")
    ax.set_xlabel("Step")
    ax.set_ylabel("Metric")
    ax.grid(True)

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig.savefig(save_path, dpi=150)

    if show:
        plt.show()
    else:
        plt.close(fig)

    return fig, axes
