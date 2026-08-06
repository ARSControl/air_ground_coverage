"""Run the proposed method and save raw data for paired baseline comparisons."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np
from scipy.interpolate import RegularGridInterpolator

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    save_archive,
    scenario_fingerprint,
)
from evaluation.multifidelity_metrics import normalized_density  # noqa: E402
from evaluation.progress import TerminalProgress  # noqa: E402
from src.coupled_config import load_coupled_configuration  # noqa: E402
from src.coupled_simulation import build_coupled_simulation  # noqa: E402


TIMING_FIELDS = (
    "low_collection",
    "aerial_control",
    "high_collection",
    "estimator_update",
    "ground_control",
    "total",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", default="configs/multifidelity.yaml")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/baseline_comparison/multifidelity/raw.npz"),
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_multifidelity_comparison(
    config_path: str | Path,
    output_path: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run full multi-fidelity episodes without computing evaluation metrics.

    The archive stores the exact simulator truth and initial conditions. A
    baseline runner must consume those arrays rather than regenerating a field
    from the seed, because different methods may consume random numbers in a
    different order.
    """
    configuration = load_coupled_configuration(config_path)
    episode_count = _positive_integer(
        configuration.aerial.num_episodes if episodes is None else episodes,
        "episodes",
    )
    step_count = _positive_integer(
        configuration.aerial.num_steps if num_steps is None else num_steps,
        "num_steps",
    )
    seeds = np.arange(
        int(configuration.aerial.random_seed),
        int(configuration.aerial.random_seed) + episode_count,
        dtype=np.int64,
    )
    total_steps = episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "initializing proposed-method episodes")

    aerial_runs: list[np.ndarray] = []
    ground_runs: list[np.ndarray] = []
    timing_runs: list[np.ndarray] = []
    truth_grid_runs: list[np.ndarray] = []
    low_truth_grid_runs: list[np.ndarray] = []
    discrepancy_grid_runs: list[np.ndarray] = []
    truth_field_runs: list[np.ndarray] = []
    truth_density_runs: list[np.ndarray] = []
    free_mask_runs: list[np.ndarray] = []
    map_runs: list[np.ndarray] = []
    posterior_episode: list[int] = []
    posterior_step: list[int] = []
    posterior_time: list[float] = []
    posterior_version: list[int] = []
    posterior_mean: list[np.ndarray] = []
    posterior_variance: list[np.ndarray] = []
    posterior_density: list[np.ndarray] = []
    posterior_low_samples: list[int] = []
    posterior_high_samples: list[int] = []
    posterior_fit_duration: list[float] = []
    posterior_prediction_duration: list[float] = []
    posterior_optimization_duration: list[float] = []
    posterior_low_only_duration: list[float] = []
    common_query: np.ndarray | None = None
    common_weights: np.ndarray | None = None
    common_query_shape: tuple[int, ...] | None = None
    aerial_fov_degrees: float | None = None
    aerial_sensing_range: float | None = None
    ground_fov_degrees: float | None = None
    ground_sensing_range: float | None = None

    for episode_index, seed in enumerate(seeds):
        simulation = build_coupled_simulation(
            configuration.aerial,
            configuration.ground,
            seed=int(seed),
        )
        if simulation.coordinator is None:
            raise RuntimeError("comparison runner requires estimator_mode=multifidelity")
        if len(simulation.aerial_team) == 0 or len(simulation.ground_team) == 0:
            raise ValueError("comparison runner requires both aerial and ground teams")

        query = np.asarray(simulation.ground_controller.query_points, dtype=float)
        weights = np.asarray(
            simulation.ground_controller.integration_weights, dtype=float
        )
        query_shape = tuple(simulation.coordinator.estimator.settings.query_shape)
        if common_query is None:
            common_query = np.array(query, copy=True)
            common_weights = np.array(weights, copy=True)
            common_query_shape = query_shape
            aerial_fov_degrees = float(configuration.aerial.fov_deg)
            aerial_sensing_range = float(configuration.aerial.fov_depth)
            ground_fov_degrees = float(configuration.ground.fov_deg)
            ground_sensing_range = float(configuration.ground.fov_depth)
        elif not (
            np.array_equal(query, common_query)
            and np.array_equal(weights, common_weights)
            and query_shape == common_query_shape
        ):
            raise ValueError("all comparison episodes must use one query grid")

        fields = simulation.coordinator.fields
        high_grid = np.asarray(fields.high, dtype=float)
        map_grid = np.asarray(simulation.ground_map, dtype=np.int8)
        truth_field = _resample_grid(high_grid, query)
        free_mask = _resample_grid(map_grid == 0, query, nearest=True).astype(bool)
        truth_density = normalized_density(
            np.maximum(truth_field, 0.0) * free_mask,
            weights,
        )
        truth_grid_runs.append(high_grid)
        low_truth_grid_runs.append(np.asarray(fields.low, dtype=float))
        discrepancy_grid_runs.append(np.asarray(fields.discrepancy, dtype=float))
        truth_field_runs.append(truth_field)
        truth_density_runs.append(truth_density)
        free_mask_runs.append(free_mask)
        map_runs.append(map_grid)

        aerial_history = [simulation.aerial_team.get_states()]
        ground_history = [simulation.ground_team.get_states()]
        step_timings: list[list[float]] = []
        last_version = 0
        for step in range(step_count):
            result = simulation.step(step)
            aerial_history.append(simulation.aerial_team.get_states())
            ground_history.append(simulation.ground_team.get_states())
            step_timings.append(
                [getattr(result.timing, field) for field in TIMING_FIELDS]
            )
            completed_steps += 1
            if progress_callback is not None:
                progress_callback(
                    completed_steps,
                    total_steps,
                    f"episode={episode_index + 1}/{episode_count} "
                    f"step={step + 1}/{step_count}",
                )
            snapshot = simulation.latest_posterior
            if snapshot is None or snapshot.version == last_version:
                continue
            last_version = snapshot.version
            posterior_episode.append(episode_index)
            posterior_step.append(step)
            posterior_time.append(float(snapshot.timestamp))
            posterior_version.append(snapshot.version)
            posterior_mean.append(np.asarray(snapshot.high_mean))
            posterior_variance.append(np.asarray(snapshot.high_variance))
            posterior_density.append(np.asarray(snapshot.density))
            posterior_low_samples.append(snapshot.low_sample_count)
            posterior_high_samples.append(snapshot.high_sample_count)
            posterior_fit_duration.append(snapshot.fit_duration)
            posterior_prediction_duration.append(snapshot.prediction_duration)
            posterior_optimization_duration.append(
                snapshot.hyperparameter_fit_duration
            )
            posterior_low_only_duration.append(snapshot.low_only_projection_duration)

        aerial_runs.append(np.asarray(aerial_history, dtype=float))
        ground_runs.append(np.asarray(ground_history, dtype=float))
        timing_runs.append(np.asarray(step_timings, dtype=float))

    if (
        common_query is None
        or common_weights is None
        or common_query_shape is None
        or aerial_fov_degrees is None
        or aerial_sensing_range is None
        or ground_fov_degrees is None
        or ground_sensing_range is None
    ):
        raise RuntimeError("comparison runner produced no episodes")
    point_count = common_query.shape[0]
    metadata = {
        "method": "multifidelity",
        "config_path": str(Path(config_path)),
        "episodes": episode_count,
        "num_steps": step_count,
        "dt": float(configuration.aerial.dt),
        "timing_fields": list(TIMING_FIELDS),
        "query_shape": list(common_query_shape),
        "resolved_aerial_config": configuration.aerial.to_dict(),
        "resolved_ground_config": configuration.ground.to_dict(),
        "scenario_contract": {
            "baseline_must_reuse_saved_truth": True,
            "baseline_must_reuse_saved_map": True,
            "baseline_must_reuse_saved_initial_states": True,
            "paired_episode_key": "seeds",
        },
    }
    arrays = {
        "method_name": np.asarray("multifidelity"),
        "seeds": seeds,
        "state_times": np.arange(step_count + 1, dtype=float)
        * float(configuration.aerial.dt),
        "query_points": common_query,
        "integration_weights": common_weights,
        "truth_field_grid": np.asarray(truth_grid_runs, dtype=float),
        "low_truth_field_grid": np.asarray(low_truth_grid_runs, dtype=float),
        "discrepancy_truth_field_grid": np.asarray(
            discrepancy_grid_runs, dtype=float
        ),
        "high_truth": np.asarray(truth_field_runs, dtype=float),
        "truth_density": np.asarray(truth_density_runs, dtype=float),
        "free_mask": np.asarray(free_mask_runs, dtype=bool),
        "map_grid": np.asarray(map_runs, dtype=np.int8),
        "aerial_states": np.asarray(aerial_runs, dtype=float),
        "ground_states": np.asarray(ground_runs, dtype=float),
        "initial_aerial_states": np.asarray(aerial_runs, dtype=float)[:, 0],
        "initial_ground_states": np.asarray(ground_runs, dtype=float)[:, 0],
        "aerial_fov_degrees": np.asarray(aerial_fov_degrees),
        "aerial_sensing_range": np.asarray(aerial_sensing_range),
        "ground_fov_degrees": np.asarray(ground_fov_degrees),
        "ground_sensing_range": np.asarray(ground_sensing_range),
        "step_timings": np.asarray(timing_runs, dtype=float),
        "posterior_episode": np.asarray(posterior_episode, dtype=np.int64),
        "posterior_step": np.asarray(posterior_step, dtype=np.int64),
        "posterior_time": np.asarray(posterior_time, dtype=float),
        "posterior_version": np.asarray(posterior_version, dtype=np.int64),
        "posterior_mean": _records(posterior_mean, point_count),
        "posterior_variance": _records(posterior_variance, point_count),
        "posterior_density": _records(posterior_density, point_count),
        "posterior_low_samples": np.asarray(posterior_low_samples, dtype=np.int64),
        "posterior_high_samples": np.asarray(posterior_high_samples, dtype=np.int64),
        "posterior_fit_duration": np.asarray(posterior_fit_duration, dtype=float),
        "posterior_prediction_duration": np.asarray(
            posterior_prediction_duration, dtype=float
        ),
        "posterior_optimization_duration": np.asarray(
            posterior_optimization_duration, dtype=float
        ),
        "posterior_low_only_duration": np.asarray(
            posterior_low_only_duration, dtype=float
        ),
    }
    metadata["scenario_fingerprint"] = scenario_fingerprint(arrays)
    return save_archive(
        output_path,
        kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _positive_integer(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if result < 1:
        raise ValueError(f"{name} must be positive")
    return result


def _resample_grid(
    grid: np.ndarray, query_points: np.ndarray, *, nearest: bool = False
) -> np.ndarray:
    values = np.asarray(grid, dtype=float)
    x_axis = np.linspace(
        float(np.min(query_points[:, 0])),
        float(np.max(query_points[:, 0])),
        values.shape[1],
    )
    y_axis = np.linspace(
        float(np.min(query_points[:, 1])),
        float(np.max(query_points[:, 1])),
        values.shape[0],
    )
    interpolator = RegularGridInterpolator(
        (y_axis, x_axis), values, method="nearest" if nearest else "linear"
    )
    return np.asarray(interpolator(query_points[:, [1, 0]]), dtype=float)


def _records(records: list[np.ndarray], width: int) -> np.ndarray:
    if not records:
        return np.empty((0, width), dtype=float)
    return np.asarray(records, dtype=float).reshape((-1, width))


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Comparison")
    try:
        path = run_multifidelity_comparison(
            arguments.config,
            arguments.output,
            episodes=arguments.episodes,
            num_steps=arguments.num_steps,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"multifidelity_comparison_raw={path}")


if __name__ == "__main__":
    main()
