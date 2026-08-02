"""Run the Egerstedt baseline on scenarios saved by the proposed method."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from time import perf_counter
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
    scenario_fingerprint,
)
from evaluation.egerstedt import (  # noqa: E402
    assign_voronoi,
    coverage_cost,
    heterogeneous_step,
    lloyd_step,
    voronoi_centroids,
)
from evaluation.progress import TerminalProgress  # noqa: E402


TIMING_FIELDS = ("aerial_control", "ground_control", "total")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        "-i",
        type=Path,
        default=Path("output/baseline_comparison/multifidelity/raw.npz"),
        help="raw proposed-method archive providing paired scenario inputs",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/baseline_comparison/egerstedt/raw.npz"),
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_egerstedt_comparison(
    scenario_path: str | Path,
    output_path: str | Path,
    *,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run the literature baseline without importing proposed-method code."""
    source_metadata, source = load_archive(
        scenario_path,
        expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    )
    fingerprint = scenario_fingerprint(source)
    recorded_fingerprint = source_metadata.get("scenario_fingerprint")
    if recorded_fingerprint != fingerprint:
        raise ValueError("scenario fingerprint does not match source archive metadata")
    _validate_scenario(source, source_metadata)

    seeds = source["seeds"]
    state_times = source["state_times"]
    query_points = source["query_points"]
    integration_weights = source["integration_weights"]
    truth_density = source["truth_density"]
    episode_count = seeds.size
    step_count = state_times.size - 1
    dt = float(source_metadata["dt"])
    total_steps = episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "loading paired Egerstedt scenarios")

    ground_range = float(source["ground_sensing_range"])
    aerial_range = float(source["aerial_sensing_range"])
    x_min = float(np.min(query_points[:, 0]))
    x_max = float(np.max(query_points[:, 0]))
    y_min = float(np.min(query_points[:, 1]))
    y_max = float(np.max(query_points[:, 1]))
    cell_area = _uniform_cell_area(query_points)

    ground_runs: list[np.ndarray] = []
    aerial_runs: list[np.ndarray] = []
    cost_runs: list[np.ndarray] = []
    sigma_runs: list[np.ndarray] = []
    timing_runs: list[np.ndarray] = []
    for episode in range(episode_count):
        ground_position = np.array(
            source["initial_ground_states"][episode, :, :2], copy=True
        )
        aerial_position = np.array(
            source["initial_aerial_states"][episode, :, :2], copy=True
        )
        density = truth_density[episode]
        ground_history = [ground_position.copy()]
        aerial_history = [aerial_position.copy()]
        costs: list[float] = []
        sigma_history: list[np.ndarray] = []
        timings: list[tuple[float, float, float]] = []

        for step in range(step_count):
            total_start = perf_counter()
            phase_start = perf_counter()
            aerial_labels, aerial_out_of_range, _ = assign_voronoi(
                query_points,
                aerial_position,
                sensing_range=aerial_range,
            )
            aerial_centroids, _ = voronoi_centroids(
                query_points,
                aerial_labels,
                np.ones(query_points.shape[0], dtype=float),
                aerial_position,
                cell_area,
                out_of_range=aerial_out_of_range,
            )
            aerial_position = lloyd_step(
                aerial_position,
                aerial_centroids,
                kappa=0.5,
                dt=dt,
            )
            aerial_duration = perf_counter() - phase_start

            phase_start = perf_counter()
            ground_labels, ground_out_of_range, _ = assign_voronoi(
                query_points,
                ground_position,
                sensing_range=ground_range,
            )
            ground_centroids, _ = voronoi_centroids(
                query_points,
                ground_labels,
                density,
                ground_position,
                cell_area,
                out_of_range=ground_out_of_range,
            )
            ground_position, sigma = heterogeneous_step(
                ground_position,
                ground_centroids,
                aerial_position,
                aerial_centroids,
                aerial_labels,
                query_points,
                density,
                ground_position.shape[0],
                dt=dt,
                aerial_oor=aerial_out_of_range,
            )
            ground_position[:, 0] = np.clip(ground_position[:, 0], x_min, x_max)
            ground_position[:, 1] = np.clip(ground_position[:, 1], y_min, y_max)
            ground_duration = perf_counter() - phase_start

            full_labels, _, _ = assign_voronoi(query_points, ground_position)
            costs.append(
                coverage_cost(
                    query_points,
                    full_labels,
                    density,
                    ground_position,
                    cell_area,
                )
            )
            sigma_history.append(np.asarray(sigma, dtype=float))
            ground_history.append(ground_position.copy())
            aerial_history.append(aerial_position.copy())
            timings.append(
                (aerial_duration, ground_duration, perf_counter() - total_start)
            )
            completed_steps += 1
            if progress_callback is not None:
                progress_callback(
                    completed_steps,
                    total_steps,
                    f"episode={episode + 1}/{episode_count} "
                    f"step={step + 1}/{step_count}",
                )

        ground_runs.append(np.asarray(ground_history, dtype=float))
        aerial_runs.append(np.asarray(aerial_history, dtype=float))
        cost_runs.append(np.asarray(costs, dtype=float))
        sigma_runs.append(np.asarray(sigma_history, dtype=float))
        timing_runs.append(np.asarray(timings, dtype=float))

    metadata = {
        "method": "egerstedt",
        "scenario_source": str(Path(scenario_path)),
        "scenario_fingerprint": fingerprint,
        "episodes": episode_count,
        "num_steps": step_count,
        "dt": dt,
        "timing_fields": list(TIMING_FIELDS),
        "algorithm_assumptions": {
            "known_truth_density": True,
            "convex_obstacle_free_domain": True,
            "ground_footprint": "omnidirectional range-limited disk",
            "aerial_controller": "uniform-density range-limited Lloyd",
            "ground_controller": "range-limited Lloyd plus hierarchical allocation",
            "kinematics": "first-order point robots",
        },
        "comparable_metrics": {
            "kl": False,
            "nrmse": False,
            "calibration_95": False,
            "coverage": True,
            "runtime": True,
        },
    }
    arrays = {
        "method_name": np.asarray("egerstedt"),
        "seeds": np.array(seeds, copy=True),
        "state_times": np.array(state_times, copy=True),
        "query_points": np.array(query_points, copy=True),
        "integration_weights": np.array(integration_weights, copy=True),
        "truth_field_grid": np.array(source["truth_field_grid"], copy=True),
        "truth_density": np.array(truth_density, copy=True),
        "free_mask": np.array(source["free_mask"], copy=True),
        "map_grid": np.array(source["map_grid"], copy=True),
        "initial_aerial_positions": np.array(
            source["initial_aerial_states"][:, :, :2], copy=True
        ),
        "initial_ground_positions": np.array(
            source["initial_ground_states"][:, :, :2], copy=True
        ),
        "initial_aerial_states": np.array(
            source["initial_aerial_states"], copy=True
        ),
        "initial_ground_states": np.array(
            source["initial_ground_states"], copy=True
        ),
        "aerial_positions": np.asarray(aerial_runs, dtype=float),
        "ground_positions": np.asarray(ground_runs, dtype=float),
        "aerial_fov_degrees": np.asarray(360.0),
        "aerial_sensing_range": np.asarray(aerial_range),
        "ground_fov_degrees": np.asarray(360.0),
        "ground_sensing_range": np.asarray(ground_range),
        "locational_cost": np.asarray(cost_runs, dtype=float),
        "sigma": np.asarray(sigma_runs, dtype=float),
        "step_timings": np.asarray(timing_runs, dtype=float),
    }
    if scenario_fingerprint(arrays) != fingerprint:
        raise RuntimeError("saved Egerstedt scenario inputs changed unexpectedly")
    return save_archive(
        output_path,
        kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_scenario(
    arrays: dict[str, np.ndarray], metadata: dict[str, object]
) -> None:
    seeds = arrays["seeds"]
    times = arrays["state_times"]
    points = arrays["query_points"]
    weights = arrays["integration_weights"]
    maps = arrays["map_grid"]
    if seeds.ndim != 1 or seeds.size < 1:
        raise ValueError("seeds must be a nonempty vector")
    if times.ndim != 1 or times.size < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError("state_times must be a strictly increasing vector")
    if int(metadata["num_steps"]) != times.size - 1:
        raise ValueError("state_times must match source num_steps")
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("query_points must have shape (N, 2)")
    if weights.shape != (points.shape[0],) or np.any(weights <= 0.0):
        raise ValueError("integration_weights must be positive and match query_points")
    if maps.shape[0] != seeds.size or np.any(maps != 0):
        raise ValueError(
            "Egerstedt baseline supports only paired obstacle-free scenarios"
        )
    expected_truth_shape = (seeds.size, points.shape[0])
    if arrays["truth_density"].shape != expected_truth_shape:
        raise ValueError("truth_density must have shape (episodes, query_points)")
    if arrays["initial_ground_states"].shape[0] != seeds.size:
        raise ValueError("initial_ground_states must match episode count")
    if arrays["initial_aerial_states"].shape[0] != seeds.size:
        raise ValueError("initial_aerial_states must match episode count")
    if arrays["initial_ground_states"].shape[1] < 1:
        raise ValueError("Egerstedt baseline requires ground robots")
    if arrays["initial_aerial_states"].shape[1] < 1:
        raise ValueError("Egerstedt baseline requires aerial robots")


def _uniform_cell_area(query_points: np.ndarray) -> float:
    x_values = np.unique(query_points[:, 0])
    y_values = np.unique(query_points[:, 1])
    if x_values.size < 2 or y_values.size < 2:
        raise ValueError("Egerstedt baseline requires a two-dimensional grid")
    if x_values.size * y_values.size != query_points.shape[0]:
        raise ValueError("Egerstedt baseline requires a Cartesian query grid")
    x_steps = np.diff(x_values)
    y_steps = np.diff(y_values)
    if not (
        np.allclose(x_steps, x_steps[0]) and np.allclose(y_steps, y_steps[0])
    ):
        raise ValueError("Egerstedt baseline requires a uniform query grid")
    return float(x_steps[0] * y_steps[0])


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Egerstedt")
    try:
        path = run_egerstedt_comparison(
            arguments.scenario,
            arguments.output,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"egerstedt_comparison_raw={path}")


if __name__ == "__main__":
    main()
