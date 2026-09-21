"""Render seven MP4 videos from one saved multi-fidelity composition episode."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import re
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import animation  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
import numpy as np
from numpy.typing import NDArray

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    RAW_ARCHIVE_KIND,
    load_archive,
)
from evaluation.multifidelity_metrics import (  # noqa: E402
    covered_probability_mass,
    footprint_normalized_coverage,
    kl_divergence,
    maximum_density_mass_for_area,
    negative_log_predictive_density,
    nrmse,
    sector_footprint_area,
)


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]
WriterFactory = Callable[[Path, int], Any]
VIDEO_NAMES = (
    "estimated_density",
    "ground_truth_density",
    "nrmse",
    "kl",
    "nlpd",
    "H",
    "effectiveness",
)
VIDEO_FILENAMES = {
    "estimated_density": "mfgp_estimated_density.mp4",
    "ground_truth_density": "ground_truth_density.mp4",
    "nrmse": "nrmse.mp4",
    "kl": "kl.mp4",
    "nlpd": "nlpd.mp4",
    "H": "H.mp4",
    "effectiveness": "effectiveness.mp4",
}
AERIAL_COLOR = "#2563EB"
GROUND_COLOR = "#EA580C"


@dataclass(frozen=True)
class EpisodeReplay:
    """Validated saved data needed to replay one completed episode."""

    composition: str
    episode: int
    seed: int
    query_shape: tuple[int, int]
    query_points: FloatArray
    integration_weights: FloatArray
    truth_field: FloatArray
    truth_density: FloatArray
    free_mask: NDArray[np.bool_]
    map_grid: NDArray[np.int8]
    aerial_states: FloatArray
    ground_states: FloatArray
    step_times: FloatArray
    posterior_steps: IntArray
    posterior_versions: IntArray
    posterior_mean: FloatArray
    posterior_variance: FloatArray
    posterior_density: FloatArray
    ground_fov_degrees: float
    ground_sensing_range: float


@dataclass(frozen=True)
class MetricSeries:
    """One metric history on completed-step indices and replay times."""

    steps: IntArray
    times: FloatArray
    values: FloatArray
    label: str
    better: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_composition_raw.npz"),
        help="saved multifidelity_composition_raw .npz archive",
    )
    parser.add_argument(
        "--composition",
        "-c",
        required=True,
        help="exact saved composition label, for example A2/G8",
    )
    parser.add_argument(
        "--episode",
        "-e",
        type=int,
        default=0,
        help="zero-based saved episode index (default: 0)",
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=None,
        help=(
            "directory for seven .mp4 files; defaults to "
            "output/multifidelity_episode_videos/<composition>/episode_NNN"
        ),
    )
    parser.add_argument("--fps", type=int, default=20, help="encoded frames per second")
    parser.add_argument(
        "--frame-interval",
        type=int,
        default=1,
        help="render every Nth completed saved step; the final step is always included",
    )
    parser.add_argument("--dpi", type=int, default=120)
    return parser.parse_args()


def load_episode_replay(
    raw_path: str | Path,
    *,
    composition: str,
    episode: int,
    require_ground_metrics: bool = True,
) -> EpisodeReplay:
    """Load and validate one episode without modifying or rerunning it."""
    metadata, values = load_archive(raw_path, expected_kind=RAW_ARCHIVE_KIND)
    return _load_episode_replay(
        metadata,
        values,
        composition=composition,
        episode=episode,
        require_ground_metrics=require_ground_metrics,
    )


def _load_episode_replay(
    metadata: dict[str, Any],
    values: dict[str, np.ndarray],
    *,
    composition: str,
    episode: int,
    require_ground_metrics: bool,
) -> EpisodeReplay:
    """Build one replay from an archive already loaded in memory."""
    if not isinstance(composition, str) or not composition.strip():
        raise TypeError("composition must be a nonempty string")
    if isinstance(episode, bool) or not isinstance(episode, int):
        raise TypeError("episode must be an integer")
    if not isinstance(require_ground_metrics, bool):
        raise TypeError("require_ground_metrics must be a boolean")
    names = tuple(str(value) for value in values["composition_names"].tolist())
    selected_name = composition.strip()
    if selected_name not in names:
        raise ValueError(
            f"unknown composition {selected_name!r}; available compositions: "
            + ", ".join(names)
        )
    composition_index = names.index(selected_name)
    episode_count = _positive_int(values["seeds"].size, "saved episode count")
    if episode < 0 or episode >= episode_count:
        raise IndexError(f"episode index {episode} is outside [0, {episode_count - 1}]")

    aerial_count = int(values["aerial_counts"][composition_index])
    ground_count = int(values["ground_counts"][composition_index])
    if require_ground_metrics and ground_count < 1:
        raise ValueError(
            "offline H and effectiveness videos require at least one ground robot"
        )
    state_times = _finite_vector(values["state_times"], "state_times")
    if state_times.size < 2 or not np.all(np.diff(state_times) > 0.0):
        raise ValueError("state_times must be strictly increasing and contain a step")
    state_count = state_times.size
    aerial_states = _selected_states(
        values, "aerial_states", composition_index, episode, aerial_count, state_count
    )
    ground_states = _selected_states(
        values, "ground_states", composition_index, episode, ground_count, state_count
    )

    query_shape = tuple(int(value) for value in metadata["query_shape"])
    if len(query_shape) != 2 or any(value < 2 for value in query_shape):
        raise ValueError("saved query_shape must describe a two-dimensional grid")
    point_count = int(np.prod(query_shape))
    query_points = np.asarray(values["query_points"], dtype=float)
    if query_points.shape != (point_count, 2) or not np.all(np.isfinite(query_points)):
        raise ValueError("query_points do not match query_shape")
    _structured_grid(query_points, query_shape)
    weights = _finite_vector(values["integration_weights"], "integration_weights")
    if (
        weights.shape != (point_count,)
        or np.any(weights < 0.0)
        or not np.any(weights > 0.0)
    ):
        raise ValueError("integration_weights must match the query grid")

    truth_field = _episode_vector(
        values["high_truth"], composition_index, episode, point_count, "high_truth"
    )
    truth_density = _episode_vector(
        values["truth_density"],
        composition_index,
        episode,
        point_count,
        "truth_density",
    )
    if np.any(truth_density < 0.0):
        raise ValueError("truth_density must be nonnegative")
    free_mask = np.asarray(values["free_mask"][composition_index, episode], dtype=bool)
    if free_mask.shape != (point_count,) or not np.any(free_mask):
        raise ValueError("free_mask must match the query grid and contain free space")
    map_grid = np.asarray(values["map_grid"][composition_index, episode], dtype=np.int8)
    if map_grid.ndim != 2 or not np.all(np.isin(map_grid, (0, 1))):
        raise ValueError("map_grid must be a two-dimensional binary array")

    selected = np.flatnonzero(
        (np.asarray(values["posterior_composition"], dtype=int) == composition_index)
        & (np.asarray(values["posterior_episode"], dtype=int) == episode)
    )
    if selected.size == 0:
        raise RuntimeError("selected composition episode has no saved posterior")
    all_steps = np.asarray(values["posterior_step"], dtype=int)
    all_versions = np.asarray(values["posterior_version"], dtype=int)
    all_times = np.asarray(values["posterior_time"], dtype=float)
    selected = selected[
        np.lexsort((all_versions[selected], all_times[selected], all_steps[selected]))
    ]
    posterior_steps = np.asarray(all_steps[selected], dtype=np.int64)
    if (
        posterior_steps[0] < 0
        or posterior_steps[-1] >= state_count - 1
        or not np.all(np.diff(posterior_steps) > 0)
    ):
        raise ValueError("saved posterior steps must be unique completed steps")
    if posterior_steps[0] != 0:
        raise ValueError("episode replay requires a posterior published at step zero")
    posterior_mean = _posterior_records(values, "posterior_mean", selected, point_count)
    posterior_variance = _posterior_records(
        values, "posterior_variance", selected, point_count
    )
    posterior_density = _posterior_records(
        values, "posterior_density", selected, point_count
    )
    if np.any(posterior_variance < 0.0) or np.any(posterior_density < 0.0):
        raise ValueError("saved posterior variance and density must be nonnegative")

    ground_config = metadata.get("resolved_ground_config")
    if not isinstance(ground_config, dict):
        raise ValueError("archive metadata is missing resolved_ground_config")
    sensor_config = ground_config.get("sensor")
    if not isinstance(sensor_config, dict):
        raise ValueError("archive metadata is missing the ground sensor configuration")
    ground_fov = _positive_float(sensor_config.get("fov_degrees"), "ground FOV")
    ground_range = _nonnegative_float(sensor_config.get("fov_depth"), "ground range")

    return EpisodeReplay(
        composition=selected_name,
        episode=episode,
        seed=int(values["seeds"][episode]),
        query_shape=query_shape,
        query_points=_readonly(query_points),
        integration_weights=_readonly(weights),
        truth_field=_readonly(truth_field),
        truth_density=_readonly(truth_density),
        free_mask=_readonly(free_mask),
        map_grid=_readonly(map_grid),
        aerial_states=_readonly(aerial_states),
        ground_states=_readonly(ground_states),
        step_times=_readonly(state_times[1:]),
        posterior_steps=_readonly(posterior_steps),
        posterior_versions=_readonly(
            np.asarray(all_versions[selected], dtype=np.int64)
        ),
        posterior_mean=_readonly(posterior_mean),
        posterior_variance=_readonly(posterior_variance),
        posterior_density=_readonly(posterior_density),
        ground_fov_degrees=ground_fov,
        ground_sensing_range=ground_range,
    )


def compute_episode_metrics(replay: EpisodeReplay) -> dict[str, MetricSeries]:
    """Compute the five requested histories from saved arrays only."""
    reconstruction = compute_reconstruction_metrics(replay)
    if replay.ground_states.shape[1] < 1:
        raise ValueError("H and effectiveness require at least one ground robot")
    free = replay.free_mask
    ground_post_step = replay.ground_states[1:, :, :3]
    h_values = np.asarray(
        [
            _locational_cost(
                replay.query_points,
                replay.truth_density,
                replay.integration_weights,
                states[:, :2],
            )
            for states in ground_post_step
        ],
        dtype=float,
    )
    free_area = float(np.sum(replay.integration_weights[free], dtype=float))
    nominal_area = ground_post_step.shape[1] * sector_footprint_area(
        fov_degrees=replay.ground_fov_degrees,
        sensing_range=replay.ground_sensing_range,
    )
    oracle_mass = maximum_density_mass_for_area(
        replay.truth_density,
        replay.integration_weights,
        area_budget=min(nominal_area, free_area),
        free_mask=free,
    )
    effectiveness = np.asarray(
        [
            footprint_normalized_coverage(
                covered_probability_mass(
                    replay.query_points,
                    replay.truth_density,
                    replay.integration_weights,
                    states,
                    fov_degrees=replay.ground_fov_degrees,
                    sensing_range=replay.ground_sensing_range,
                ),
                oracle_mass,
            )
            for states in ground_post_step
        ],
        dtype=float,
    )
    all_steps = np.arange(replay.step_times.size, dtype=np.int64)
    return {
        **reconstruction,
        "H": _metric(
            all_steps,
            replay.step_times,
            h_values,
            r"Locational cost $H$",
            "lower is better",
        ),
        "effectiveness": _metric(
            all_steps,
            replay.step_times,
            effectiveness,
            "Footprint-normalized effectiveness",
            "higher is better",
        ),
    }


def compute_reconstruction_metrics(replay: EpisodeReplay) -> dict[str, MetricSeries]:
    """Compute posterior reconstruction histories, including homogeneous teams."""
    free = replay.free_mask
    posterior_times = replay.step_times[replay.posterior_steps]
    nrmse_values = np.asarray(
        [nrmse(replay.truth_field[free], mean[free]) for mean in replay.posterior_mean],
        dtype=float,
    )
    kl_values = np.asarray(
        [
            kl_divergence(replay.truth_density, density, replay.integration_weights)
            for density in replay.posterior_density
        ],
        dtype=float,
    )
    nlpd_values = np.asarray(
        [
            negative_log_predictive_density(
                replay.truth_field[free],
                mean[free],
                variance[free],
                replay.integration_weights[free],
            )
            for mean, variance in zip(
                replay.posterior_mean, replay.posterior_variance, strict=True
            )
        ],
        dtype=float,
    )

    return {
        "nrmse": _metric(
            replay.posterior_steps,
            posterior_times,
            nrmse_values,
            "NRMSE",
            "lower is better",
        ),
        "kl": _metric(
            replay.posterior_steps,
            posterior_times,
            kl_values,
            r"$D_{KL}(truth\,||\,estimate)$",
            "lower is better",
        ),
        "nlpd": _metric(
            replay.posterior_steps,
            posterior_times,
            nlpd_values,
            "NLPD",
            "lower is better",
        ),
    }


def render_episode_videos(
    raw_path: str | Path,
    *,
    composition: str,
    episode: int,
    output_directory: str | Path | None = None,
    fps: int = 20,
    frame_interval: int = 1,
    dpi: int = 120,
    writer_factory: WriterFactory | None = None,
    progress: Callable[[str], None] | None = None,
) -> dict[str, Path]:
    """Write the two density and five metric MP4s for one saved episode."""
    replay = load_episode_replay(raw_path, composition=composition, episode=episode)
    fps = _positive_int(fps, "fps")
    frame_interval = _positive_int(frame_interval, "frame_interval")
    dpi = _minimum_int(dpi, "dpi", 72)
    safe_composition = re.sub(r"[^A-Za-z0-9_.-]+", "_", replay.composition).strip("_")
    destination = (
        Path("output/multifidelity_episode_videos")
        / safe_composition
        / f"episode_{episode:03d}"
        if output_directory is None
        else Path(output_directory)
    )
    if not destination.name:
        raise ValueError("output_directory must name a directory")
    destination.mkdir(parents=True, exist_ok=True)
    outputs = {name: destination / VIDEO_FILENAMES[name] for name in VIDEO_NAMES}
    factory = _default_mp4_writer if writer_factory is None else writer_factory
    frame_steps = _frame_steps(replay.step_times.size, frame_interval)
    metrics = compute_episode_metrics(replay)

    _render_density_video(
        replay,
        frame_steps,
        outputs["estimated_density"],
        fps,
        dpi,
        factory,
        estimated=True,
    )
    _emit(progress, f"rendered estimated_density={outputs['estimated_density']}")
    _render_density_video(
        replay,
        frame_steps,
        outputs["ground_truth_density"],
        fps,
        dpi,
        factory,
        estimated=False,
    )
    _emit(progress, f"rendered ground_truth_density={outputs['ground_truth_density']}")
    for name in VIDEO_NAMES[2:]:
        _render_metric_video(
            replay,
            metrics[name],
            frame_steps,
            outputs[name],
            fps,
            dpi,
            factory,
        )
        _emit(progress, f"rendered {name}={outputs[name]}")
    return outputs


def _render_density_video(
    replay: EpisodeReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
    *,
    estimated: bool,
) -> None:
    grid_x, grid_y = _structured_grid(replay.query_points, replay.query_shape)
    values = replay.posterior_density if estimated else replay.truth_density[None, :]
    maximum = max(float(np.max(values)), 1.0e-12)
    figure, axis = plt.subplots(figsize=(7.2, 6.0), constrained_layout=True)
    writer = factory(output, fps)
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                if estimated:
                    record = int(
                        np.searchsorted(replay.posterior_steps, step, side="right") - 1
                    )
                    field = replay.posterior_density[record]
                    version_text = f"; posterior v{replay.posterior_versions[record]}"
                    title = "MFGP estimated controller density"
                else:
                    field = replay.truth_density
                    version_text = ""
                    title = "Hidden HIGH-fidelity ground-truth density"
                image = axis.pcolormesh(
                    grid_x,
                    grid_y,
                    field.reshape(replay.query_shape),
                    shading="auto",
                    cmap="magma",
                    vmin=0.0,
                    vmax=maximum,
                )
                _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
                _plot_robots(
                    axis,
                    replay.aerial_states[int(step) + 1, :, :2],
                    replay.ground_states[int(step) + 1, :, :2],
                )
                axis.set_title(
                    f"{title}\nstep={int(step)}, t={replay.step_times[int(step)]:g} s"
                    f"{version_text}"
                )
                _format_map_axis(axis, grid_x, grid_y)
                colorbar = figure.colorbar(image, ax=axis, label="probability density")
                writer.grab_frame()
                colorbar.remove()
    finally:
        plt.close(figure)


def _render_metric_video(
    replay: EpisodeReplay,
    metric: MetricSeries,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
) -> None:
    figure, axis = plt.subplots(figsize=(8.0, 4.5), constrained_layout=True)
    writer = factory(output, fps)
    ymin, ymax = _metric_limits(metric.values)
    end_time = float(replay.step_times[-1])
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                included = metric.steps <= step
                times = metric.times[included]
                values = metric.values[included]
                axis.plot(times, values, color="#2563EB", linewidth=2.4)
                if values.size:
                    axis.scatter(
                        times[-1],
                        values[-1],
                        color="#EA580C",
                        edgecolor="white",
                        linewidth=0.7,
                        s=48,
                        zorder=3,
                    )
                    current = f"{values[-1]:.6g}"
                else:
                    current = "not yet available"
                axis.axvline(
                    replay.step_times[int(step)],
                    color="#555555",
                    linestyle="--",
                    alpha=0.55,
                )
                axis.set_xlim(0.0, end_time)
                axis.set_ylim(ymin, ymax)
                axis.set_xlabel("saved replay time [s]")
                axis.set_ylabel(metric.label)
                axis.set_title(
                    f"{metric.label}: {current}\n"
                    f"{replay.composition}, episode {replay.episode}, "
                    f"step={int(step)} ({metric.better})"
                )
                axis.grid(alpha=0.25)
                writer.grab_frame()
    finally:
        plt.close(figure)


def _default_mp4_writer(path: Path, fps: int) -> Any:
    if path.suffix.lower() != ".mp4":
        raise ValueError("offline episode videos must use the .mp4 suffix")
    if not animation.FFMpegWriter.isAvailable():
        try:
            import imageio_ffmpeg

            matplotlib.rcParams["animation.ffmpeg_path"] = (
                imageio_ffmpeg.get_ffmpeg_exe()
            )
        except (ImportError, RuntimeError):
            pass
    if not animation.FFMpegWriter.isAvailable():
        raise RuntimeError(
            "MP4 encoding requires imageio-ffmpeg or an ffmpeg executable visible "
            "to Matplotlib"
        )
    return animation.FFMpegWriter(
        fps=fps,
        codec="libx264",
        metadata={"artist": "ral_marta offline multi-fidelity replay"},
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )


def _selected_states(
    values: dict[str, np.ndarray],
    key: str,
    composition: int,
    episode: int,
    robot_count: int,
    state_count: int,
) -> FloatArray:
    states = np.asarray(values[key][composition, episode], dtype=float)
    if (
        states.ndim != 3
        or states.shape[0] != state_count
        or states.shape[1] < robot_count
        or states.shape[2] < 3
    ):
        raise ValueError(f"saved {key} has an incompatible shape")
    selected = states[:, :robot_count]
    if not np.all(np.isfinite(selected)):
        raise ValueError(f"active saved {key} entries must be finite")
    return selected


def _episode_vector(
    values: np.ndarray,
    composition: int,
    episode: int,
    point_count: int,
    name: str,
) -> FloatArray:
    vector = np.asarray(values[composition, episode], dtype=float)
    if vector.shape != (point_count,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"saved {name} must match the query grid")
    return vector


def _posterior_records(
    values: dict[str, np.ndarray],
    name: str,
    selected: IntArray,
    point_count: int,
) -> FloatArray:
    records = np.asarray(values[name][selected], dtype=float)
    if records.shape != (selected.size, point_count) or not np.all(
        np.isfinite(records)
    ):
        raise ValueError(f"saved {name} records must match the query grid")
    return records


def _structured_grid(
    query_points: FloatArray, query_shape: tuple[int, int]
) -> tuple[FloatArray, FloatArray]:
    grid = np.asarray(query_points, dtype=float).reshape((*query_shape, 2))
    grid_x = grid[..., 0]
    grid_y = grid[..., 1]
    expected_x, expected_y = np.meshgrid(grid_x[0], grid_y[:, 0])
    if not np.allclose(grid_x, expected_x) or not np.allclose(grid_y, expected_y):
        raise ValueError("query_points must form a structured row-major grid")
    return grid_x, grid_y


def _locational_cost(
    query_points: FloatArray,
    truth_density: FloatArray,
    integration_weights: FloatArray,
    ground_positions: FloatArray,
) -> float:
    squared_distances = np.sum(
        (query_points[:, None, :] - ground_positions[None, :, :]) ** 2,
        axis=2,
    )
    return float(
        np.sum(
            np.min(squared_distances, axis=1) * truth_density * integration_weights,
            dtype=float,
        )
    )


def _metric(
    steps: IntArray,
    times: FloatArray,
    values: FloatArray,
    label: str,
    better: str,
) -> MetricSeries:
    return MetricSeries(
        steps=_readonly(np.asarray(steps, dtype=np.int64)),
        times=_readonly(np.asarray(times, dtype=float)),
        values=_readonly(np.asarray(values, dtype=float)),
        label=label,
        better=better,
    )


def _frame_steps(step_count: int, interval: int) -> IntArray:
    steps = np.arange(0, step_count, interval, dtype=np.int64)
    if steps[-1] != step_count - 1:
        steps = np.append(steps, step_count - 1)
    return steps


def _metric_limits(values: FloatArray) -> tuple[float, float]:
    minimum = float(np.min(values))
    maximum = float(np.max(values))
    if np.isclose(minimum, maximum):
        padding = max(abs(minimum), 1.0) * 0.08
    else:
        padding = (maximum - minimum) * 0.08
    return minimum - padding, maximum + padding


def _plot_robots(axis, aerial: FloatArray, ground: FloatArray) -> None:
    if aerial.size:
        axis.scatter(
            aerial[:, 0],
            aerial[:, 1],
            marker="^",
            s=62,
            color=AERIAL_COLOR,
            edgecolor="white",
            linewidth=0.8,
            label="aerial",
            zorder=4,
        )
    if ground.size:
        axis.scatter(
            ground[:, 0],
            ground[:, 1],
            marker="o",
            s=50,
            color=GROUND_COLOR,
            edgecolor="white",
            linewidth=0.8,
            label="ground",
            zorder=4,
        )
    axis.legend(loc="upper right", fontsize=8, framealpha=0.86)


def _plot_obstacles(axis, map_grid, grid_x, grid_y) -> None:
    if not np.any(map_grid):
        return
    obstacle = np.ma.masked_where(map_grid == 0, map_grid)
    axis.imshow(
        obstacle,
        origin="lower",
        extent=(
            float(np.min(grid_x)),
            float(np.max(grid_x)),
            float(np.min(grid_y)),
            float(np.max(grid_y)),
        ),
        cmap=ListedColormap(["#202020"]),
        vmin=0,
        vmax=1,
        interpolation="nearest",
        alpha=0.78,
        zorder=2,
    )


def _format_map_axis(axis, grid_x, grid_y) -> None:
    axis.set_xlabel("x [map units]")
    axis.set_ylabel("y [map units]")
    axis.set_xlim(float(np.min(grid_x)), float(np.max(grid_x)))
    axis.set_ylim(float(np.min(grid_y)), float(np.max(grid_y)))
    axis.set_aspect("equal", adjustable="box")
    axis.grid(color="white", alpha=0.15, linewidth=0.6)


def _finite_vector(values: Any, name: str) -> FloatArray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite vector")
    return array


def _readonly(values: NDArray[Any]) -> NDArray[Any]:
    result = np.array(values, copy=True)
    result.setflags(write=False)
    return result


def _positive_int(value: Any, name: str) -> int:
    return _minimum_int(value, name, 1)


def _minimum_int(value: Any, name: str, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if result < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return result


def _positive_float(value: Any, name: str) -> float:
    result = _nonnegative_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result


def _emit(callback: Callable[[str], None] | None, message: str) -> None:
    if callback is not None:
        callback(message)


def main() -> None:
    arguments = parse_args()
    outputs = render_episode_videos(
        arguments.input,
        composition=arguments.composition,
        episode=arguments.episode,
        output_directory=arguments.output_dir,
        fps=arguments.fps,
        frame_interval=arguments.frame_interval,
        dpi=arguments.dpi,
        progress=lambda message: print(message, flush=True),
    )
    print(f"videos={len(outputs)}")


if __name__ == "__main__":
    main()
