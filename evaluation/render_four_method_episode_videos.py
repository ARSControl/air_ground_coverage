"""Render ten MP4s and four PNGs from a saved four-method episode."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.cm import ScalarMappable  # noqa: E402
from matplotlib.colors import ListedColormap, Normalize  # noqa: E402
import numpy as np
from numpy.typing import NDArray

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    SCENARIO_ARRAYS,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    scenario_fingerprint,
)
from evaluation.multifidelity_metrics import (  # noqa: E402
    kl_divergence,
    negative_log_predictive_density,
    nrmse,
)
from evaluation.plot_four_method_comparison import (  # noqa: E402
    COLORS,
    LABELS,
    LINESTYLES,
    METHODS,
)
from evaluation.plot_four_method_trajectories import (  # noqa: E402
    AERIAL_COLOR,
    GROUND_COLOR,
)
from evaluation.render_multifidelity_episode_videos import (  # noqa: E402
    _default_mp4_writer,
)


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]
WriterFactory = Callable[[Path, int], Any]
VIDEO_NAMES = (
    "estimated_density",
    "variance",
    "density_difference",
    "spatial_comparison",
    "density_comparison",
    "nrmse",
    "kl",
    "nlpd",
    "L",
    "effectiveness",
)
VIDEO_FILENAMES = {
    "estimated_density": "mfgp_estimated_density.mp4",
    "variance": "mfgp_variance.mp4",
    "density_difference": "mfgp_density_difference.mp4",
    "spatial_comparison": "mfgp_spatial_reconstruction_comparison.mp4",
    "density_comparison": "four_method_density_comparison.mp4",
    "nrmse": "mfgp_nrmse.mp4",
    "kl": "mfgp_kl.mp4",
    "nlpd": "mfgp_nlpd.mp4",
    "L": "four_method_L.mp4",
    "effectiveness": "four_method_effectiveness.mp4",
}
IMAGE_NAMES = (
    "ground_truth_density",
    "estimated_density_trajectories",
    "variance_trajectories",
    "density_difference_trajectories",
)
IMAGE_FILENAMES = {
    "ground_truth_density": "ground_truth_density.png",
    "estimated_density_trajectories": "mfgp_trajectories_estimated_density.png",
    "variance_trajectories": "mfgp_trajectories_variance.png",
    "density_difference_trajectories": ("mfgp_trajectories_density_difference.png"),
}
DENSITY_COLORMAP = "Reds"
VARIANCE_COLORMAP = "Greys"
DENSITY_DIFFERENCE_COLORMAP = "RdBu_r"


@dataclass(frozen=True)
class MetricSeries:
    """One MFGP posterior metric history."""

    steps: IntArray
    times: FloatArray
    values: FloatArray
    label: str


@dataclass(frozen=True)
class FourMethodReplay:
    """Validated synchronized data for one paired four-method episode."""

    method_names: tuple[str, ...]
    episode: int
    seed: int
    query_shape: tuple[int, int]
    query_points: FloatArray
    integration_weights: FloatArray
    truth_field: FloatArray
    truth_density: FloatArray
    free_mask: NDArray[np.bool_]
    map_grid: NDArray[np.int8]
    aerial_positions: tuple[FloatArray, ...]
    ground_positions: tuple[FloatArray, ...]
    state_times: FloatArray
    step_times: FloatArray
    posterior_steps: IntArray
    posterior_versions: IntArray
    posterior_mean: FloatArray
    posterior_variance: FloatArray
    posterior_density: FloatArray
    effectiveness: FloatArray
    locational_cost: FloatArray

    @property
    def comparison_metrics(self) -> dict[str, FloatArray]:
        return {"L": self.locational_cost, "effectiveness": self.effectiveness}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("output/four_method_comparison"),
        help="directory containing the four raw archives and evaluated.npz",
    )
    parser.add_argument(
        "--episode",
        "-e",
        type=int,
        default=0,
        help="zero-based paired episode index (default: 0)",
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=None,
        help=(
            "directory for ten .mp4 and four .png files; defaults to "
            "<input-dir>/videos/episode_NNN"
        ),
    )
    parser.add_argument("--fps", type=int, default=20, help="encoded frames per second")
    parser.add_argument(
        "--frame-interval",
        type=int,
        default=1,
        help="render every Nth completed step; the final step is always included",
    )
    parser.add_argument("--dpi", type=int, default=120)
    return parser.parse_args()


def load_four_method_replay(
    input_directory: str | Path,
    *,
    episode: int,
) -> FourMethodReplay:
    """Load one paired episode and validate all five comparison archives."""
    if isinstance(episode, bool) or not isinstance(episode, int):
        raise TypeError("episode must be an integer")
    root = Path(input_directory)
    metadata: list[dict[str, object]] = []
    raw: list[dict[str, np.ndarray]] = []
    for relative, kind in (
        ("multifidelity/raw.npz", MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND),
        ("egerstedt_lr/raw.npz", EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND),
        ("egerstedt_full/raw.npz", EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND),
        ("zhang/raw.npz", ZHANG_COMPARISON_RAW_ARCHIVE_KIND),
    ):
        item_metadata, values = load_archive(root / relative, expected_kind=kind)
        metadata.append(item_metadata)
        raw.append(values)
    evaluated_metadata, evaluated = load_archive(
        root / "evaluated.npz",
        expected_kind=FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    )
    _validate_archives(metadata, raw, evaluated_metadata, evaluated, episode)

    proposed = raw[0]
    state_times = _finite_vector(proposed["state_times"], "state_times")
    point_count = proposed["query_points"].shape[0]
    query_shape = tuple(int(value) for value in metadata[0]["query_shape"])
    if len(query_shape) != 2 or int(np.prod(query_shape)) != point_count:
        raise ValueError("query_shape does not match the saved comparison grid")
    query_points = np.asarray(proposed["query_points"], dtype=float)
    _structured_grid(query_points, query_shape)
    weights = _finite_vector(proposed["integration_weights"], "integration_weights")
    truth_field = _episode_vector(
        proposed["high_truth"], episode, point_count, "high_truth"
    )
    truth_density = _episode_vector(
        proposed["truth_density"], episode, point_count, "truth_density"
    )
    free_mask = np.asarray(proposed["free_mask"][episode], dtype=bool)
    if free_mask.shape != (point_count,) or not np.any(free_mask):
        raise ValueError("free_mask must match the comparison grid")
    map_grid = np.asarray(proposed["map_grid"][episode], dtype=np.int8)
    if map_grid.ndim != 2 or not np.all(np.isin(map_grid, (0, 1))):
        raise ValueError("map_grid must be a two-dimensional binary array")

    aerial_positions = (
        _positions(proposed["aerial_states"][episode, ..., :2], state_times.size),
        _positions(raw[1]["aerial_positions"][episode], state_times.size),
        _positions(raw[2]["aerial_positions"][episode], state_times.size),
        _positions(raw[3]["aerial_positions"][episode], state_times.size),
    )
    ground_positions = (
        _positions(proposed["ground_states"][episode, ..., :2], state_times.size),
        _positions(raw[1]["ground_positions"][episode], state_times.size),
        _positions(raw[2]["ground_positions"][episode], state_times.size),
        _positions(raw[3]["ground_positions"][episode], state_times.size),
    )

    selected = np.flatnonzero(
        np.asarray(proposed["posterior_episode"], dtype=int) == episode
    )
    if selected.size == 0:
        raise RuntimeError("selected episode has no MFGP posterior records")
    all_steps = np.asarray(proposed["posterior_step"], dtype=int)
    all_versions = np.asarray(proposed["posterior_version"], dtype=int)
    all_times = np.asarray(proposed["posterior_time"], dtype=float)
    selected = selected[
        np.lexsort((all_versions[selected], all_times[selected], all_steps[selected]))
    ]
    posterior_steps = np.asarray(all_steps[selected], dtype=np.int64)
    if (
        posterior_steps[0] != 0
        or posterior_steps[-1] >= state_times.size - 1
        or not np.all(np.diff(posterior_steps) > 0)
    ):
        raise ValueError("MFGP posterior steps are incompatible with the replay")
    posterior_mean = _records(proposed, "posterior_mean", selected, point_count)
    posterior_variance = _records(proposed, "posterior_variance", selected, point_count)
    posterior_density = _records(proposed, "posterior_density", selected, point_count)
    if np.any(posterior_variance < 0.0) or np.any(posterior_density < 0.0):
        raise ValueError("MFGP posterior variance and density must be nonnegative")

    method_names = tuple(str(value) for value in evaluated["method_names"].tolist())
    effectiveness = np.asarray(evaluated["coverage"][:, episode], dtype=float)
    locational_cost = np.asarray(evaluated["locational_cost"][:, episode], dtype=float)
    expected_metric_shape = (len(METHODS), state_times.size)
    if (
        effectiveness.shape != expected_metric_shape
        or locational_cost.shape != expected_metric_shape
        or not np.all(np.isfinite(effectiveness))
        or not np.all(np.isfinite(locational_cost))
    ):
        raise ValueError("evaluated four-method histories have incompatible shapes")

    return FourMethodReplay(
        method_names=method_names,
        episode=episode,
        seed=int(proposed["seeds"][episode]),
        query_shape=query_shape,
        query_points=_readonly(query_points),
        integration_weights=_readonly(weights),
        truth_field=_readonly(truth_field),
        truth_density=_readonly(truth_density),
        free_mask=_readonly(free_mask),
        map_grid=_readonly(map_grid),
        aerial_positions=tuple(_readonly(value) for value in aerial_positions),
        ground_positions=tuple(_readonly(value) for value in ground_positions),
        state_times=_readonly(state_times),
        step_times=_readonly(state_times[1:]),
        posterior_steps=_readonly(posterior_steps),
        posterior_versions=_readonly(
            np.asarray(all_versions[selected], dtype=np.int64)
        ),
        posterior_mean=_readonly(posterior_mean),
        posterior_variance=_readonly(posterior_variance),
        posterior_density=_readonly(posterior_density),
        effectiveness=_readonly(effectiveness),
        locational_cost=_readonly(locational_cost),
    )


def compute_reconstruction_metrics(
    replay: FourMethodReplay,
) -> dict[str, MetricSeries]:
    """Compute MFGP-only reconstruction metrics from the proposed raw archive."""
    free = replay.free_mask
    times = replay.step_times[replay.posterior_steps]
    values = {
        "nrmse": np.asarray(
            [
                nrmse(replay.truth_field[free], mean[free])
                for mean in replay.posterior_mean
            ],
            dtype=float,
        ),
        "kl": np.asarray(
            [
                kl_divergence(replay.truth_density, density, replay.integration_weights)
                for density in replay.posterior_density
            ],
            dtype=float,
        ),
        "nlpd": np.asarray(
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
        ),
    }
    labels = {
        "nrmse": "MFGP NRMSE",
        "kl": r"MFGP $D_{KL}(truth\,||\,estimate)$",
        "nlpd": "MFGP NLPD",
    }
    return {
        name: MetricSeries(
            steps=_readonly(replay.posterior_steps),
            times=_readonly(times),
            values=_readonly(metric_values),
            label=labels[name],
        )
        for name, metric_values in values.items()
    }


def static_image_fields(replay: FourMethodReplay) -> dict[str, FloatArray]:
    """Return the immutable fields used by the three static companion images."""
    return {
        "ground_truth_density": replay.truth_density,
        "estimated_density_trajectories": replay.posterior_density[-1],
        "variance_trajectories": replay.posterior_variance[-1],
        "density_difference_trajectories": (
            replay.posterior_density[-1] - replay.truth_density
        ),
    }


def mfgp_frame_fields(
    replay: FourMethodReplay,
    step: int,
) -> dict[str, FloatArray]:
    """Return the latest causal MFGP spatial fields for one replay step."""
    if isinstance(step, bool) or not isinstance(step, (int, np.integer)):
        raise TypeError("step must be an integer")
    step_value = int(step)
    if step_value < 0 or step_value >= replay.step_times.size:
        raise IndexError("step is outside the saved replay")
    record = int(np.searchsorted(replay.posterior_steps, step_value, side="right") - 1)
    density = replay.posterior_density[record]
    return {
        "density": density,
        "variance": replay.posterior_variance[record],
        "density_difference": density - replay.truth_density,
    }


def comparison_spatial_fields(
    replay: FourMethodReplay,
    step: int,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """Return truth, estimate, variance, and signed error for one frame."""
    fields = mfgp_frame_fields(replay, step)
    return (
        replay.truth_density,
        fields["density"],
        fields["variance"],
        fields["density_difference"],
    )


def render_four_method_episode_images(
    input_directory: str | Path,
    *,
    episode: int,
    output_directory: str | Path | None = None,
    dpi: int = 120,
) -> dict[str, Path]:
    """Write truth and proposed final-posterior trajectory companion PNGs."""
    replay = load_four_method_replay(input_directory, episode=episode)
    dpi = _minimum_int(dpi, "dpi", 72)
    destination = _output_directory(input_directory, episode, output_directory)
    return _render_static_images(replay, destination, dpi)


def render_four_method_episode_videos(
    input_directory: str | Path,
    *,
    episode: int,
    output_directory: str | Path | None = None,
    fps: int = 20,
    frame_interval: int = 1,
    dpi: int = 120,
    writer_factory: WriterFactory | None = None,
    progress: Callable[[str], None] | None = None,
) -> dict[str, Path]:
    """Write five map and five metric MP4s for one paired comparison episode."""
    replay = load_four_method_replay(input_directory, episode=episode)
    fps = _positive_int(fps, "fps")
    frame_interval = _positive_int(frame_interval, "frame_interval")
    dpi = _minimum_int(dpi, "dpi", 72)
    destination = _output_directory(input_directory, episode, output_directory)
    outputs = {name: destination / VIDEO_FILENAMES[name] for name in VIDEO_NAMES}
    factory = _default_mp4_writer if writer_factory is None else writer_factory
    frame_steps = _frame_steps(replay.step_times.size, frame_interval)
    reconstruction = compute_reconstruction_metrics(replay)

    _render_estimated_density(
        replay, frame_steps, outputs["estimated_density"], fps, dpi, factory
    )
    _emit(progress, f"rendered estimated_density={outputs['estimated_density']}")
    _render_mfgp_spatial_video(
        replay,
        frame_steps,
        outputs["variance"],
        fps,
        dpi,
        factory,
        field="variance",
        title="Proposed MFGP posterior variance",
        colorbar_label="latent posterior variance",
        cmap=VARIANCE_COLORMAP,
        minimum=0.0,
        maximum=max(float(np.max(replay.posterior_variance)), 1.0e-12),
    )
    _emit(progress, f"rendered variance={outputs['variance']}")
    differences = replay.posterior_density - replay.truth_density[None, :]
    difference_maximum = max(float(np.max(np.abs(differences))), 1.0e-12)
    _render_mfgp_spatial_video(
        replay,
        frame_steps,
        outputs["density_difference"],
        fps,
        dpi,
        factory,
        field="density_difference",
        title="Proposed MFGP density difference (estimate − truth)",
        colorbar_label="estimated − ground-truth density",
        cmap=DENSITY_DIFFERENCE_COLORMAP,
        minimum=-difference_maximum,
        maximum=difference_maximum,
    )
    _emit(
        progress,
        f"rendered density_difference={outputs['density_difference']}",
    )
    _render_spatial_comparison(
        replay,
        frame_steps,
        outputs["spatial_comparison"],
        fps,
        dpi,
        factory,
    )
    _emit(
        progress,
        f"rendered spatial_comparison={outputs['spatial_comparison']}",
    )
    _render_density_comparison(
        replay, frame_steps, outputs["density_comparison"], fps, dpi, factory
    )
    _emit(progress, f"rendered density_comparison={outputs['density_comparison']}")
    for name in ("nrmse", "kl", "nlpd"):
        _render_reconstruction_metric(
            replay,
            reconstruction[name],
            frame_steps,
            outputs[name],
            fps,
            dpi,
            factory,
        )
        _emit(progress, f"rendered {name}={outputs[name]}")
    for name, label in (
        ("L", r"Locational cost $\mathcal{L}$"),
        (
            "effectiveness",
            r"Footprint-normalized effectiveness $\mathcal{E}$",
        ),
    ):
        _render_comparison_metric(
            replay,
            replay.comparison_metrics[name],
            label,
            frame_steps,
            outputs[name],
            fps,
            dpi,
            factory,
        )
        _emit(progress, f"rendered {name}={outputs[name]}")
    image_outputs = _render_static_images(replay, destination, dpi)
    for name, output in image_outputs.items():
        _emit(progress, f"rendered {name}={output}")
    return outputs


def _output_directory(
    input_directory: str | Path,
    episode: int,
    output_directory: str | Path | None,
) -> Path:
    destination = (
        Path(input_directory) / "videos" / f"episode_{episode:03d}"
        if output_directory is None
        else Path(output_directory)
    )
    if not destination.name:
        raise ValueError("output_directory must name a directory")
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _render_static_images(
    replay: FourMethodReplay,
    destination: Path,
    dpi: int,
) -> dict[str, Path]:
    outputs = {name: destination / IMAGE_FILENAMES[name] for name in IMAGE_NAMES}
    fields = static_image_fields(replay)
    density_maximum = max(
        float(np.max(replay.truth_density)),
        float(np.max(replay.posterior_density)),
        1.0e-12,
    )
    _render_static_field(
        replay,
        fields["ground_truth_density"],
        outputs["ground_truth_density"],
        title="Ground-truth density",
        colorbar_label="probability density",
        cmap=DENSITY_COLORMAP,
        minimum=0.0,
        maximum=density_maximum,
        trajectories=False,
        dpi=dpi,
    )
    _render_static_field(
        replay,
        fields["estimated_density_trajectories"],
        outputs["estimated_density_trajectories"],
        title="Proposed trajectories over final MFGP estimate",
        colorbar_label="probability density",
        cmap=DENSITY_COLORMAP,
        minimum=0.0,
        maximum=density_maximum,
        trajectories=True,
        dpi=dpi,
    )
    _render_static_field(
        replay,
        fields["variance_trajectories"],
        outputs["variance_trajectories"],
        title="Proposed trajectories over final MFGP variance",
        colorbar_label="latent posterior variance",
        cmap=VARIANCE_COLORMAP,
        minimum=0.0,
        maximum=max(float(np.max(fields["variance_trajectories"])), 1.0e-12),
        trajectories=True,
        dpi=dpi,
    )
    difference = fields["density_difference_trajectories"]
    difference_maximum = max(float(np.max(np.abs(difference))), 1.0e-12)
    _render_static_field(
        replay,
        difference,
        outputs["density_difference_trajectories"],
        title="Proposed trajectories over MFGP density difference",
        colorbar_label="estimated − ground-truth density",
        cmap=DENSITY_DIFFERENCE_COLORMAP,
        minimum=-difference_maximum,
        maximum=difference_maximum,
        trajectories=True,
        dpi=dpi,
    )
    return outputs


def _render_static_field(
    replay: FourMethodReplay,
    field: FloatArray,
    output: Path,
    *,
    title: str,
    colorbar_label: str,
    cmap: str,
    minimum: float,
    maximum: float,
    trajectories: bool,
    dpi: int,
) -> None:
    grid_x, grid_y = _structured_grid(replay.query_points, replay.query_shape)
    figure, axis = plt.subplots(figsize=(7.2, 6.0), constrained_layout=True)
    image = axis.pcolormesh(
        grid_x,
        grid_y,
        field.reshape(replay.query_shape),
        shading="auto",
        cmap=cmap,
        vmin=minimum,
        vmax=maximum,
    )
    _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
    if trajectories:
        _plot_trajectory_team(
            axis,
            replay.aerial_positions[0],
            replay.ground_positions[0],
        )
    axis.set_title(title)
    _format_map_axis(axis, grid_x, grid_y)
    figure.colorbar(image, ax=axis, label=colorbar_label)
    figure.savefig(output, dpi=dpi, facecolor="white")
    plt.close(figure)


def _plot_trajectory_team(axis, aerial: FloatArray, ground: FloatArray) -> None:
    for robot in range(aerial.shape[1]):
        axis.plot(
            aerial[:, robot, 0],
            aerial[:, robot, 1],
            linestyle="--",
            color=AERIAL_COLOR,
            linewidth=1.4,
            alpha=0.9,
            label="aerial trajectory" if robot == 0 else None,
            zorder=3,
        )
    for robot in range(ground.shape[1]):
        axis.plot(
            ground[:, robot, 0],
            ground[:, robot, 1],
            color=GROUND_COLOR,
            linewidth=1.4,
            alpha=0.86,
            label="ground trajectory" if robot == 0 else None,
            zorder=3,
        )
    axis.scatter(
        aerial[0, :, 0],
        aerial[0, :, 1],
        marker="x",
        color=AERIAL_COLOR,
        linewidth=1.2,
        zorder=4,
    )
    axis.scatter(
        ground[0, :, 0],
        ground[0, :, 1],
        marker="x",
        color=GROUND_COLOR,
        linewidth=1.2,
        zorder=4,
    )
    axis.scatter(
        aerial[-1, :, 0],
        aerial[-1, :, 1],
        marker="^",
        s=54,
        color=AERIAL_COLOR,
        edgecolor="white",
        linewidth=0.7,
        zorder=4,
    )
    axis.scatter(
        ground[-1, :, 0],
        ground[-1, :, 1],
        marker="o",
        s=42,
        color=GROUND_COLOR,
        edgecolor="white",
        linewidth=0.7,
        zorder=4,
    )
    axis.legend(loc="upper right", fontsize=8, framealpha=0.86)


def _render_estimated_density(
    replay: FourMethodReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
) -> None:
    _render_mfgp_spatial_video(
        replay,
        frame_steps,
        output,
        fps,
        dpi,
        factory,
        field="density",
        title="Proposed MFGP estimated controller density",
        colorbar_label="probability density",
        cmap=DENSITY_COLORMAP,
        minimum=0.0,
        maximum=max(float(np.max(replay.posterior_density)), 1.0e-12),
    )


def _render_mfgp_spatial_video(
    replay: FourMethodReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
    *,
    field: str,
    title: str,
    colorbar_label: str,
    cmap: str,
    minimum: float,
    maximum: float,
) -> None:
    grid_x, grid_y = _structured_grid(replay.query_points, replay.query_shape)
    figure, axis = plt.subplots(figsize=(7.2, 6.0), constrained_layout=True)
    writer = factory(output, fps)
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                fields = mfgp_frame_fields(replay, int(step))
                image = axis.pcolormesh(
                    grid_x,
                    grid_y,
                    fields[field].reshape(replay.query_shape),
                    shading="auto",
                    cmap=cmap,
                    vmin=minimum,
                    vmax=maximum,
                )
                _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
                _plot_current_robots(
                    axis,
                    replay.aerial_positions[0][int(step) + 1],
                    replay.ground_positions[0][int(step) + 1],
                )
                axis.set_title(title)
                _format_map_axis(axis, grid_x, grid_y)
                colorbar = figure.colorbar(image, ax=axis, label=colorbar_label)
                writer.grab_frame()
                colorbar.remove()
    finally:
        plt.close(figure)


def _render_spatial_comparison(
    replay: FourMethodReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
) -> None:
    grid_x, grid_y = _structured_grid(replay.query_points, replay.query_shape)
    density_maximum = max(
        float(np.max(replay.truth_density)),
        float(np.max(replay.posterior_density)),
        1.0e-12,
    )
    variance_maximum = max(float(np.max(replay.posterior_variance)), 1.0e-12)
    differences = replay.posterior_density - replay.truth_density[None, :]
    difference_maximum = max(float(np.max(np.abs(differences))), 1.0e-12)
    density_norm = Normalize(vmin=0.0, vmax=density_maximum)
    variance_norm = Normalize(vmin=0.0, vmax=variance_maximum)
    difference_norm = Normalize(
        vmin=-difference_maximum,
        vmax=difference_maximum,
    )
    norms = (density_norm, density_norm, variance_norm, difference_norm)
    colormaps = (
        DENSITY_COLORMAP,
        DENSITY_COLORMAP,
        VARIANCE_COLORMAP,
        DENSITY_DIFFERENCE_COLORMAP,
    )
    titles = (
        "Ground truth",
        "MFGP estimate",
        "MFGP variance",
        "MFGP density difference (estimate − truth)",
    )
    figure, axes = plt.subplots(2, 2, figsize=(9.2, 8.0), constrained_layout=True)
    figure.suptitle("Proposed MFGP spatial reconstruction")
    figure.colorbar(
        ScalarMappable(norm=density_norm, cmap=DENSITY_COLORMAP),
        ax=axes[0, :],
        label="probability density",
        fraction=0.025,
        pad=0.02,
    )
    figure.colorbar(
        ScalarMappable(norm=variance_norm, cmap=VARIANCE_COLORMAP),
        ax=axes[1, 0],
        label="latent posterior variance",
        fraction=0.05,
        pad=0.02,
    )
    figure.colorbar(
        ScalarMappable(norm=difference_norm, cmap=DENSITY_DIFFERENCE_COLORMAP),
        ax=axes[1, 1],
        label="estimated − truth density",
        fraction=0.05,
        pad=0.02,
    )
    writer = factory(output, fps)
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                fields = comparison_spatial_fields(replay, int(step))
                for axis, field, norm, cmap, title in zip(
                    axes.ravel(),
                    fields,
                    norms,
                    colormaps,
                    titles,
                    strict=True,
                ):
                    axis.clear()
                    axis.pcolormesh(
                        grid_x,
                        grid_y,
                        field.reshape(replay.query_shape),
                        shading="auto",
                        cmap=cmap,
                        norm=norm,
                    )
                    _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
                    _plot_current_robots(
                        axis,
                        replay.aerial_positions[0][int(step) + 1],
                        replay.ground_positions[0][int(step) + 1],
                        legend=False,
                    )
                    axis.set_title(title)
                    _format_map_axis(axis, grid_x, grid_y)
                writer.grab_frame()
    finally:
        plt.close(figure)


def comparison_density_fields(
    replay: FourMethodReplay, step: int
) -> tuple[FloatArray, ...]:
    """Return proposed MFGP density followed by three paired truth fields."""
    if isinstance(step, bool) or not isinstance(step, (int, np.integer)):
        raise TypeError("step must be an integer")
    step_value = int(step)
    if step_value < 0 or step_value >= replay.step_times.size:
        raise IndexError("step is outside the saved replay")
    record = int(np.searchsorted(replay.posterior_steps, step_value, side="right") - 1)
    return (replay.posterior_density[record],) + (replay.truth_density,) * 3


def _render_density_comparison(
    replay: FourMethodReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
) -> None:
    grid_x, grid_y = _structured_grid(replay.query_points, replay.query_shape)
    maximum = max(
        float(np.max(replay.truth_density)),
        float(np.max(replay.posterior_density)),
        1.0e-12,
    )
    figure, axes = plt.subplots(2, 2, figsize=(9.2, 8.0), constrained_layout=True)
    writer = factory(output, fps)
    labels = _method_labels()
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                fields = comparison_density_fields(replay, int(step))
                for method, axis in enumerate(axes.ravel()):
                    axis.clear()
                    axis.pcolormesh(
                        grid_x,
                        grid_y,
                        fields[method].reshape(replay.query_shape),
                        shading="auto",
                        cmap=DENSITY_COLORMAP,
                        vmin=0.0,
                        vmax=maximum,
                    )
                    _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
                    _plot_current_robots(
                        axis,
                        replay.aerial_positions[method][int(step) + 1],
                        replay.ground_positions[method][int(step) + 1],
                        legend=False,
                    )
                    field_label = "MFGP estimate" if method == 0 else "ground truth"
                    axis.set_title(
                        f"{labels[replay.method_names[method]]} — {field_label}"
                    )
                    _format_map_axis(axis, grid_x, grid_y)
                figure.suptitle("Four-method density comparison")
                writer.grab_frame()
    finally:
        plt.close(figure)


def _render_reconstruction_metric(
    replay: FourMethodReplay,
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
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                included = metric.steps <= step
                times = metric.times[included]
                values = metric.values[included]
                axis.plot(
                    times, values, color=COLORS["multifidelity_lr"], linewidth=2.4
                )
                axis.scatter(
                    times[-1],
                    values[-1],
                    color="#EA580C",
                    edgecolor="white",
                    linewidth=0.7,
                    s=48,
                    zorder=3,
                )
                _format_metric_axis(
                    axis,
                    replay,
                    int(step),
                    ymin,
                    ymax,
                    metric.label,
                    metric.label,
                )
                writer.grab_frame()
    finally:
        plt.close(figure)


def _render_comparison_metric(
    replay: FourMethodReplay,
    histories: FloatArray,
    label: str,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
) -> None:
    figure, axis = plt.subplots(figsize=(8.0, 4.5), constrained_layout=True)
    writer = factory(output, fps)
    ymin, ymax = _metric_limits(histories)
    labels = _method_labels()
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                stop = int(step) + 2
                for method_index, method in enumerate(replay.method_names):
                    axis.plot(
                        replay.state_times[:stop],
                        histories[method_index, :stop],
                        color=COLORS[method],
                        linestyle=LINESTYLES[method],
                        linewidth=2.0,
                        label=labels[method],
                    )
                _format_metric_axis(
                    axis,
                    replay,
                    int(step),
                    ymin,
                    ymax,
                    label,
                    "",
                )
                axis.legend(loc="best", fontsize=8)
                writer.grab_frame()
    finally:
        plt.close(figure)


def _format_metric_axis(
    axis,
    replay: FourMethodReplay,
    step: int,
    ymin: float,
    ymax: float,
    title: str,
    ylabel: str,
) -> None:
    axis.axvline(replay.step_times[step], color="#555555", linestyle="--", alpha=0.55)
    axis.set_xlim(0.0, float(replay.step_times[-1]))
    axis.set_ylim(ymin, ymax)
    axis.set_xlabel("paired mission time [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(alpha=0.25)


def _validate_archives(
    metadata: list[dict[str, object]],
    raw: list[dict[str, np.ndarray]],
    evaluated_metadata: dict[str, object],
    evaluated: dict[str, np.ndarray],
    episode: int,
) -> None:
    fingerprints = [scenario_fingerprint(values) for values in raw]
    recorded = [values.get("scenario_fingerprint") for values in metadata]
    evaluated_fingerprint = evaluated_metadata.get("scenario_fingerprint")
    if len(set((*fingerprints, *recorded, evaluated_fingerprint))) != 1:
        raise ValueError("four-method inputs do not share one scenario fingerprint")
    for name in SCENARIO_ARRAYS:
        if not all(np.array_equal(raw[0][name], values[name]) for values in raw[1:]):
            raise ValueError(f"four-method inputs differ in scenario array {name!r}")
    episode_count = raw[0]["seeds"].size
    if episode < 0 or episode >= episode_count:
        raise IndexError(f"episode index {episode} is outside [0, {episode_count - 1}]")
    if tuple(evaluated_metadata.get("methods", ())) != METHODS:
        raise ValueError("evaluated archive does not declare the four expected methods")
    if not np.array_equal(evaluated["seeds"], raw[0]["seeds"]):
        raise ValueError("evaluated and raw archives have different episodes")
    if not np.array_equal(evaluated["state_times"], raw[0]["state_times"]):
        raise ValueError("evaluated and raw archives have different state times")


def _positions(values: np.ndarray, state_count: int) -> FloatArray:
    positions = np.asarray(values, dtype=float)
    if (
        positions.ndim != 3
        or positions.shape[0] != state_count
        or positions.shape[1] < 1
        or positions.shape[2] != 2
        or not np.all(np.isfinite(positions))
    ):
        raise ValueError("saved robot positions have an incompatible shape")
    return positions


def _episode_vector(
    values: np.ndarray, episode: int, point_count: int, name: str
) -> FloatArray:
    vector = np.asarray(values[episode], dtype=float)
    if vector.shape != (point_count,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"saved {name} must match the comparison grid")
    return vector


def _records(
    values: dict[str, np.ndarray], name: str, selected: IntArray, width: int
) -> FloatArray:
    records = np.asarray(values[name][selected], dtype=float)
    if records.shape != (selected.size, width) or not np.all(np.isfinite(records)):
        raise ValueError(f"saved {name} records have an incompatible shape")
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


def _plot_current_robots(axis, aerial, ground, *, legend: bool = True) -> None:
    axis.scatter(
        aerial[:, 0],
        aerial[:, 1],
        marker="^",
        s=54,
        color=AERIAL_COLOR,
        edgecolor="white",
        linewidth=0.7,
        label="aerial",
        zorder=4,
    )
    axis.scatter(
        ground[:, 0],
        ground[:, 1],
        marker="o",
        s=42,
        color=GROUND_COLOR,
        edgecolor="white",
        linewidth=0.7,
        label="ground",
        zorder=4,
    )
    if legend:
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
    axis.grid(color="white", alpha=0.14, linewidth=0.6)


def _method_labels() -> dict[str, str]:
    labels = dict(LABELS)
    labels["multifidelity_lr"] = "Proposed MFGP-LR"
    return labels


def _frame_steps(step_count: int, interval: int) -> IntArray:
    steps = np.arange(0, step_count, interval, dtype=np.int64)
    if steps[-1] != step_count - 1:
        steps = np.append(steps, step_count - 1)
    return steps


def _metric_limits(values: FloatArray) -> tuple[float, float]:
    minimum = float(np.min(values))
    maximum = float(np.max(values))
    padding = (
        max(abs(minimum), 1.0) * 0.08
        if np.isclose(minimum, maximum)
        else (maximum - minimum) * 0.08
    )
    return minimum - padding, maximum + padding


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


def _emit(callback: Callable[[str], None] | None, message: str) -> None:
    if callback is not None:
        callback(message)


def main() -> None:
    arguments = parse_args()
    outputs = render_four_method_episode_videos(
        arguments.input_dir,
        episode=arguments.episode,
        output_directory=arguments.output_dir,
        fps=arguments.fps,
        frame_interval=arguments.frame_interval,
        dpi=arguments.dpi,
        progress=lambda message: print(message, flush=True),
    )
    print(f"videos={len(outputs)} images={len(IMAGE_NAMES)}")


if __name__ == "__main__":
    main()
