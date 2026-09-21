"""Render homogeneous-baseline reconstruction comparisons from one saved episode."""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.cm import ScalarMappable  # noqa: E402
from matplotlib.colors import Normalize  # noqa: E402
import numpy as np
from numpy.typing import NDArray

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    RAW_ARCHIVE_KIND,
    load_archive,
)
from evaluation.render_multifidelity_episode_videos import (  # noqa: E402
    EpisodeReplay,
    MetricSeries,
    WriterFactory,
    _default_mp4_writer,
    _emit,
    _format_map_axis,
    _frame_steps,
    _load_episode_replay,
    _minimum_int,
    _plot_obstacles,
    _plot_robots,
    _positive_int,
    _structured_grid,
    compute_reconstruction_metrics,
)


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int64]
VIDEO_NAMES = (
    "estimated_density_comparison",
    "ground_truth_density_comparison",
    "nrmse",
    "kl",
    "nlpd",
)
VIDEO_FILENAMES = {
    "estimated_density_comparison": "homogeneous_baseline_estimated_density.mp4",
    "ground_truth_density_comparison": "homogeneous_baseline_ground_truth_density.mp4",
    "nrmse": "homogeneous_baseline_nrmse.mp4",
    "kl": "homogeneous_baseline_kl.mp4",
    "nlpd": "homogeneous_baseline_nlpd.mp4",
}
METHOD_COLORS = ("#2563EB", "#D99A00", "#159D76")
METHOD_LINESTYLES = ("-", "--", "-")


@dataclass(frozen=True)
class HomogeneousBaselineReplay:
    """One heterogeneous episode paired with both homogeneous endpoints."""

    replays: tuple[EpisodeReplay, EpisodeReplay, EpisodeReplay]
    labels: tuple[str, str, str]

    @property
    def episode(self) -> int:
        return self.replays[0].episode

    @property
    def seed(self) -> int:
        return self.replays[0].seed

    @property
    def step_times(self) -> FloatArray:
        return self.replays[0].step_times


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
        help="heterogeneous composition to compare, for example A2/G8",
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
            "directory for five .mp4 files; defaults to "
            "output/homogeneous_baseline_episode_videos/<composition>/episode_NNN"
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


def load_homogeneous_baseline_replay(
    raw_path: str | Path,
    *,
    composition: str,
    episode: int,
) -> HomogeneousBaselineReplay:
    """Load a heterogeneous episode and its equal-size homogeneous endpoints."""
    metadata, values = load_archive(raw_path, expected_kind=RAW_ARCHIVE_KIND)
    names = tuple(str(value) for value in values["composition_names"].tolist())
    selected_name = composition.strip() if isinstance(composition, str) else composition
    if selected_name not in names:
        raise ValueError(
            f"unknown composition {selected_name!r}; available compositions: "
            + ", ".join(names)
        )
    aerial_counts = np.asarray(values["aerial_counts"], dtype=int)
    ground_counts = np.asarray(values["ground_counts"], dtype=int)
    if aerial_counts.shape != (len(names),) or ground_counts.shape != (len(names),):
        raise ValueError("saved robot counts must match composition_names")
    selected = names.index(selected_name)
    aerial_count = int(aerial_counts[selected])
    ground_count = int(ground_counts[selected])
    if aerial_count < 1 or ground_count < 1:
        raise ValueError("the selected proposed composition must be heterogeneous")
    total = aerial_count + ground_count
    aerial_matches = np.flatnonzero((aerial_counts == total) & (ground_counts == 0))
    ground_matches = np.flatnonzero((aerial_counts == 0) & (ground_counts == total))
    if aerial_matches.size != 1 or ground_matches.size != 1:
        raise ValueError(
            "archive must contain exactly one equal-size aerial-only and ground-only "
            "baseline"
        )
    indices = (selected, int(aerial_matches[0]), int(ground_matches[0]))
    replays = tuple(
        _load_episode_replay(
            metadata,
            values,
            composition=names[index],
            episode=episode,
            require_ground_metrics=False,
        )
        for index in indices
    )
    _validate_pairing(replays)
    labels = (
        f"Proposed {replays[0].composition}",
        f"Aerial-only {replays[1].composition}",
        f"Ground-only {replays[2].composition}",
    )
    return HomogeneousBaselineReplay(replays=replays, labels=labels)


def comparison_reconstruction_metrics(
    comparison: HomogeneousBaselineReplay,
) -> dict[str, tuple[MetricSeries, MetricSeries, MetricSeries]]:
    """Compute NRMSE, KL, and NLPD for all three saved posterior streams."""
    by_method = tuple(
        compute_reconstruction_metrics(replay) for replay in comparison.replays
    )
    return {
        name: tuple(metrics[name] for metrics in by_method)
        for name in ("nrmse", "kl", "nlpd")
    }


def comparison_density_fields(
    comparison: HomogeneousBaselineReplay,
    *,
    step: int,
    estimated: bool,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    """Return each method's latest estimate or its validated paired truth."""
    if isinstance(step, bool) or not isinstance(step, (int, np.integer)):
        raise TypeError("step must be an integer")
    step_index = int(step)
    if step_index < 0 or step_index >= comparison.step_times.size:
        raise IndexError(
            f"step {step_index} is outside [0, {comparison.step_times.size - 1}]"
        )
    if not isinstance(estimated, bool):
        raise TypeError("estimated must be a boolean")
    if not estimated:
        truth = comparison.replays[0].truth_density
        return truth, truth, truth
    fields: list[FloatArray] = []
    for replay in comparison.replays:
        record = int(
            np.searchsorted(replay.posterior_steps, step_index, side="right") - 1
        )
        if record < 0:
            raise RuntimeError(
                "no causal posterior is available for the requested step"
            )
        fields.append(replay.posterior_density[record])
    return fields[0], fields[1], fields[2]


def render_homogeneous_baseline_episode_videos(
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
    """Write two density comparisons and three live reconstruction MP4s."""
    comparison = load_homogeneous_baseline_replay(
        raw_path,
        composition=composition,
        episode=episode,
    )
    fps = _positive_int(fps, "fps")
    frame_interval = _positive_int(frame_interval, "frame_interval")
    dpi = _minimum_int(dpi, "dpi", 72)
    safe_composition = re.sub(r"[^A-Za-z0-9_.-]+", "_", composition).strip("_")
    destination = (
        Path("output/homogeneous_baseline_episode_videos")
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
    frame_steps = _frame_steps(comparison.step_times.size, frame_interval)

    _render_density_comparison(
        comparison,
        frame_steps,
        outputs["estimated_density_comparison"],
        fps,
        dpi,
        factory,
        estimated=True,
    )
    _emit(
        progress,
        f"rendered estimated_density_comparison="
        f"{outputs['estimated_density_comparison']}",
    )
    _render_density_comparison(
        comparison,
        frame_steps,
        outputs["ground_truth_density_comparison"],
        fps,
        dpi,
        factory,
        estimated=False,
    )
    _emit(
        progress,
        f"rendered ground_truth_density_comparison="
        f"{outputs['ground_truth_density_comparison']}",
    )
    metrics = comparison_reconstruction_metrics(comparison)
    for name in ("nrmse", "kl", "nlpd"):
        _render_metric_comparison(
            comparison,
            metrics[name],
            frame_steps,
            outputs[name],
            fps,
            dpi,
            factory,
            metric=name,
        )
        _emit(progress, f"rendered {name}={outputs[name]}")
    return outputs


def _validate_pairing(
    replays: tuple[EpisodeReplay, EpisodeReplay, EpisodeReplay],
) -> None:
    proposed = replays[0]
    paired_arrays = (
        "step_times",
        "query_points",
        "integration_weights",
        "truth_field",
        "truth_density",
        "free_mask",
        "map_grid",
    )
    for baseline in replays[1:]:
        if baseline.episode != proposed.episode or baseline.seed != proposed.seed:
            raise ValueError("comparison episodes must share an episode index and seed")
        for name in paired_arrays:
            if not np.array_equal(getattr(baseline, name), getattr(proposed, name)):
                raise ValueError(f"comparison episodes do not share {name}")


def _render_density_comparison(
    comparison: HomogeneousBaselineReplay,
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
    *,
    estimated: bool,
) -> None:
    proposed = comparison.replays[0]
    grid_x, grid_y = _structured_grid(proposed.query_points, proposed.query_shape)
    maximum = (
        max(float(np.max(replay.posterior_density)) for replay in comparison.replays)
        if estimated
        else float(np.max(proposed.truth_density))
    )
    norm = Normalize(vmin=0.0, vmax=max(maximum, 1.0e-12))
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(12.0, 4.2),
        constrained_layout=True,
    )
    figure.suptitle(
        "Estimated HIGH-density comparison"
        if estimated
        else "Ground-truth density comparison"
    )
    colorbar = figure.colorbar(
        ScalarMappable(norm=norm, cmap="magma"),
        ax=axes,
        label="probability density",
        fraction=0.025,
        pad=0.02,
    )
    writer = factory(output, fps)
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                fields = comparison_density_fields(
                    comparison, step=int(step), estimated=estimated
                )
                for axis, replay, label, field in zip(
                    axes,
                    comparison.replays,
                    comparison.labels,
                    fields,
                    strict=True,
                ):
                    axis.clear()
                    axis.pcolormesh(
                        grid_x,
                        grid_y,
                        field.reshape(proposed.query_shape),
                        shading="auto",
                        cmap="magma",
                        norm=norm,
                    )
                    _plot_obstacles(axis, replay.map_grid, grid_x, grid_y)
                    _plot_robots(
                        axis,
                        replay.aerial_states[int(step) + 1, :, :2],
                        replay.ground_states[int(step) + 1, :, :2],
                    )
                    axis.set_title(label)
                    _format_map_axis(axis, grid_x, grid_y)
                writer.grab_frame()
    finally:
        colorbar.remove()
        plt.close(figure)


def _render_metric_comparison(
    comparison: HomogeneousBaselineReplay,
    series_by_method: tuple[MetricSeries, MetricSeries, MetricSeries],
    frame_steps: IntArray,
    output: Path,
    fps: int,
    dpi: int,
    factory: WriterFactory,
    *,
    metric: str,
) -> None:
    titles = {"nrmse": "NRMSE", "kl": "KL divergence", "nlpd": "NLPD"}
    labels = {"nrmse": "NRMSE", "kl": r"$D_{KL}$", "nlpd": "NLPD"}
    all_values = np.concatenate([series.values for series in series_by_method])
    ymin, ymax = _metric_limits(all_values)
    figure, axis = plt.subplots(figsize=(8.0, 4.5), constrained_layout=True)
    writer = factory(output, fps)
    try:
        with writer.saving(figure, output, dpi=dpi):
            for step in frame_steps:
                axis.clear()
                for label, series, color, linestyle in zip(
                    comparison.labels,
                    series_by_method,
                    METHOD_COLORS,
                    METHOD_LINESTYLES,
                    strict=True,
                ):
                    included = series.steps <= step
                    times = series.times[included]
                    values = series.values[included]
                    axis.plot(
                        times,
                        values,
                        color=color,
                        linestyle=linestyle,
                        linewidth=2.4,
                        label=label,
                    )
                    if values.size:
                        axis.scatter(
                            times[-1],
                            values[-1],
                            color=color,
                            edgecolor="white",
                            linewidth=0.7,
                            s=42,
                            zorder=3,
                        )
                axis.axvline(
                    comparison.step_times[int(step)],
                    color="#555555",
                    linestyle=":",
                    alpha=0.55,
                )
                axis.set_xlim(0.0, float(comparison.step_times[-1]))
                axis.set_ylim(ymin, ymax)
                axis.set_xlabel("saved replay time [s]")
                axis.set_ylabel(labels[metric])
                axis.set_title(titles[metric])
                axis.grid(alpha=0.25)
                axis.legend(loc="best", fontsize=8, framealpha=0.86)
                writer.grab_frame()
    finally:
        plt.close(figure)


def _metric_limits(values: FloatArray) -> tuple[float, float]:
    minimum = float(np.min(values))
    maximum = float(np.max(values))
    padding = (
        max(abs(minimum), 1.0) * 0.08
        if np.isclose(minimum, maximum)
        else (maximum - minimum) * 0.08
    )
    return minimum - padding, maximum + padding


def main() -> None:
    arguments = parse_args()
    outputs = render_homogeneous_baseline_episode_videos(
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
