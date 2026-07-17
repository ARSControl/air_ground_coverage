"""Independent visualization recorder for coupled multi-fidelity runs."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import AbstractContextManager
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation  # noqa: E402
import numpy as np


WriterFactory = Callable[[Path, int], Any]


class MultifidelityVideoRecorder:
    """Render selected completed simulation states without affecting the loop."""

    def __init__(
        self,
        output_path: str | Path,
        *,
        fps: int,
        frame_interval: int,
        writer_factory: WriterFactory | None = None,
    ) -> None:
        self.output_path = Path(output_path)
        if not self.output_path.name:
            raise ValueError("output_path must name a file")
        self.fps = _positive_int(fps, "fps")
        self.frame_interval = _positive_int(frame_interval, "frame_interval")
        self._writer_factory = writer_factory or _default_writer
        self._figure = None
        self._axes = None
        self._writer_context: AbstractContextManager[Any] | None = None
        self._writer: Any | None = None
        self._last_captured_step: int | None = None
        self._uncertainty_vmax: float | None = None

    @property
    def uncertainty_vmax(self) -> float | None:
        """Fixed uncertainty color limit chosen from the first saved frame."""
        return self._uncertainty_vmax

    def capture(
        self,
        step_num: int,
        simulation_time: float,
        snapshot: Any | None,
        aerial_positions: np.ndarray,
        ground_positions: np.ndarray,
        *,
        force: bool = False,
    ) -> bool:
        """Render one already-computed state, returning whether a frame was saved."""
        if isinstance(step_num, bool) or not isinstance(step_num, int) or step_num < 0:
            raise ValueError("step_num must be a nonnegative integer")
        if not np.isfinite(simulation_time):
            raise ValueError("simulation_time must be finite")
        if snapshot is None:
            return False
        if self._last_captured_step == step_num:
            return False
        if not force and step_num % self.frame_interval:
            return False
        self._open_if_needed()
        self._draw_frame(
            float(simulation_time), snapshot, aerial_positions, ground_positions
        )
        self._writer.grab_frame()
        self._last_captured_step = step_num
        return True

    def close(self) -> Path | None:
        """Finalize the video writer and release plotting resources."""
        if self._writer_context is None:
            return None
        try:
            self._writer_context.__exit__(None, None, None)
        finally:
            plt.close(self._figure)
            self._writer_context = None
            self._writer = None
            self._figure = None
            self._axes = None
        return self.output_path

    def _open_if_needed(self) -> None:
        if self._writer is not None:
            return
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._figure, self._axes = plt.subplots(1, 2, figsize=(10, 4.5), constrained_layout=True)
        self._writer = self._writer_factory(self.output_path, self.fps)
        self._writer_context = self._writer.saving(
            self._figure, self.output_path, dpi=160
        )
        self._writer_context.__enter__()

    def _draw_frame(
        self,
        simulation_time: float,
        snapshot: Any,
        aerial_positions: np.ndarray,
        ground_positions: np.ndarray,
    ) -> None:
        grid_x, grid_y = _structured_grid(snapshot)
        density = _grid_values(snapshot.density, snapshot.query_shape, "density")
        uncertainty = np.sqrt(
            np.maximum(_grid_values(snapshot.high_variance, snapshot.query_shape, "high_variance"), 0.0)
        )
        if self._uncertainty_vmax is None:
            self._uncertainty_vmax = max(float(np.max(uncertainty)), 1.0e-12)
        aerial = _positions(aerial_positions, "aerial_positions")
        ground = _positions(ground_positions, "ground_positions")
        density_axis, uncertainty_axis = self._axes
        for axis in self._axes:
            axis.clear()

        density_axis.pcolormesh(
            grid_x, grid_y, density, shading="auto", cmap="magma"
        )
        uncertainty_axis.pcolormesh(
            grid_x,
            grid_y,
            uncertainty,
            shading="auto",
            cmap="viridis",
            vmin=0.0,
            vmax=self._uncertainty_vmax,
        )
        self._plot_positions(density_axis, aerial, ground)
        self._plot_positions(uncertainty_axis, aerial, ground)
        density_axis.set_title("Controller density")
        uncertainty_axis.set_title(
            "HIGH posterior standard deviation "
            f"(fixed 0–{self._uncertainty_vmax:.3g})"
        )
        for axis in self._axes:
            axis.set_xlabel("x")
            axis.set_ylabel("y")
            axis.set_aspect("equal", adjustable="box")
            axis.set_xlim(float(np.min(grid_x)), float(np.max(grid_x)))
            axis.set_ylim(float(np.min(grid_y)), float(np.max(grid_y)))
            axis.grid(True, color="white", alpha=0.18, linewidth=0.7)
        self._figure.suptitle(
            "Multi-fidelity coupled simulation "
            f"t={simulation_time:.2f}; posterior v{snapshot.version}"
        )

    @staticmethod
    def _plot_positions(axis, aerial: np.ndarray, ground: np.ndarray) -> None:
        if aerial.size:
            axis.scatter(
                aerial[:, 0], aerial[:, 1], marker="^", s=54, color="#2563eb",
                edgecolor="white", linewidth=0.7, label="aerial",
            )
        if ground.size:
            axis.scatter(
                ground[:, 0], ground[:, 1], marker="o", s=42, color="#f97316",
                edgecolor="white", linewidth=0.7, label="ground",
            )
        if aerial.size or ground.size:
            axis.legend(loc="upper right", fontsize=8)


def _default_writer(path: Path, fps: int) -> Any:
    suffix = path.suffix.lower()
    if suffix == ".gif":
        return animation.PillowWriter(fps=fps)
    if suffix not in {".mp4", ".m4v", ".mov", ".avi"}:
        raise ValueError("video_path must use .mp4, .m4v, .mov, .avi, or .gif")
    if not animation.FFMpegWriter.isAvailable():
        raise RuntimeError(
            "MP4 video encoding requires an ffmpeg executable visible to Matplotlib; "
            "install ffmpeg or configure visualization.video_path with a .gif suffix"
        )
    return animation.FFMpegWriter(fps=fps)


def _structured_grid(snapshot: Any) -> tuple[np.ndarray, np.ndarray]:
    query_shape = tuple(snapshot.query_shape)
    if len(query_shape) != 2:
        raise ValueError("video frames require a two-dimensional posterior grid")
    points = np.asarray(snapshot.query_points, dtype=float)
    if points.shape != (int(np.prod(query_shape)), 2):
        raise ValueError("posterior query_points do not match query_shape")
    grid = points.reshape((*query_shape, 2))
    grid_x, grid_y = grid[..., 0], grid[..., 1]
    expected_x, expected_y = np.meshgrid(grid_x[0, :], grid_y[:, 0])
    if not np.allclose(grid_x, expected_x) or not np.allclose(grid_y, expected_y):
        raise ValueError("video frames require a structured posterior grid")
    return grid_x, grid_y


def _grid_values(values: Any, query_shape: tuple[int, int], name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (int(np.prod(query_shape)),):
        raise ValueError(f"{name} must match posterior query_shape")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return array.reshape(query_shape)


def _positions(values: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return np.empty((0, 2), dtype=float)
    if array.ndim != 2 or array.shape[1] != 2 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must have shape (N, 2) and be finite")
    return array


def _positive_int(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value
