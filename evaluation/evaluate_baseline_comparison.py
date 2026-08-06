"""Evaluate common coverage and runtime metrics from paired method archives."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND,
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
    scenario_fingerprint,
)
from evaluation.multifidelity_metrics import (  # noqa: E402
    covered_probability_mass,
    footprint_normalized_coverage,
    maximum_density_mass_for_area,
    sector_footprint_area,
)
from evaluation.progress import TerminalProgress  # noqa: E402


METHOD_NAMES = np.asarray(("multifidelity", "egerstedt"))
TIMING_STATISTICS = ("mean", "median", "p95", "maximum")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--multifidelity",
        type=Path,
        default=Path("output/baseline_comparison/multifidelity/raw.npz"),
    )
    parser.add_argument(
        "--egerstedt",
        type=Path,
        default=Path("output/baseline_comparison/egerstedt/raw.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/baseline_comparison/evaluated.npz"),
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    plot_group = parser.add_mutually_exclusive_group()
    plot_group.add_argument(
        "--plot-output",
        type=Path,
        help=(
            "coverage/runtime PNG path; defaults to coverage_metrics.png beside "
            "the evaluated archive"
        ),
    )
    plot_group.add_argument(
        "--no-plot",
        action="store_true",
        help="save only the evaluated archive",
    )
    return parser.parse_args()


def evaluate_baseline_comparison(
    multifidelity_path: str | Path,
    egerstedt_path: str | Path,
    output_path: str | Path,
    *,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute common metrics from saved outputs without rerunning either method."""
    multifidelity_metadata, multifidelity = load_archive(
        multifidelity_path,
        expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    )
    egerstedt_metadata, egerstedt = load_archive(
        egerstedt_path,
        expected_kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    )
    fingerprint = _validate_pair(
        multifidelity_metadata,
        multifidelity,
        egerstedt_metadata,
        egerstedt,
    )

    seeds = multifidelity["seeds"]
    state_times = multifidelity["state_times"]
    episode_count = seeds.size
    state_count = state_times.size
    method_count = METHOD_NAMES.size
    total_work = method_count * episode_count * (state_count + 1)
    completed_work = 0
    if progress_callback is not None:
        progress_callback(0, total_work, "validated paired raw archives")

    method_states = (
        _multifidelity_ground_states(multifidelity, episode_count, state_count),
        _egerstedt_ground_states(egerstedt, episode_count, state_count),
    )
    fov_degrees = np.asarray(
        (
            _scalar(multifidelity["ground_fov_degrees"], "ground_fov_degrees"),
            _scalar(egerstedt["ground_fov_degrees"], "ground_fov_degrees"),
        ),
        dtype=float,
    )
    sensing_ranges = np.asarray(
        (
            _scalar(multifidelity["ground_sensing_range"], "ground_sensing_range"),
            _scalar(egerstedt["ground_sensing_range"], "ground_sensing_range"),
        ),
        dtype=float,
    )

    coverage_shape = (method_count, episode_count, state_count)
    covered_mass = np.empty(coverage_shape, dtype=float)
    coverage = np.empty(coverage_shape, dtype=float)
    oracle_mass = np.empty((method_count, episode_count), dtype=float)
    area_budget = np.empty((method_count, episode_count), dtype=float)
    free_area = np.empty(episode_count, dtype=float)
    area_fraction = np.empty((method_count, episode_count), dtype=float)

    points = multifidelity["query_points"]
    weights = multifidelity["integration_weights"]
    truth_density = multifidelity["truth_density"]
    free_masks = multifidelity["free_mask"].astype(bool)
    for episode in range(episode_count):
        free_area[episode] = float(np.sum(weights[free_masks[episode]], dtype=float))
        if free_area[episode] <= 0.0:
            raise ValueError("each episode must contain positive free-space area")

    for method, states_by_episode in enumerate(method_states):
        for episode in range(episode_count):
            robot_count = states_by_episode.shape[2]
            nominal_area = robot_count * sector_footprint_area(
                fov_degrees=float(fov_degrees[method]),
                sensing_range=float(sensing_ranges[method]),
            )
            budget = min(nominal_area, free_area[episode])
            maximum_mass = maximum_density_mass_for_area(
                truth_density[episode],
                weights,
                area_budget=budget,
                free_mask=free_masks[episode],
            )
            area_budget[method, episode] = budget
            area_fraction[method, episode] = budget / free_area[episode]
            oracle_mass[method, episode] = maximum_mass
            completed_work += 1
            if progress_callback is not None:
                progress_callback(
                    completed_work,
                    total_work,
                    f"coverage oracle method={METHOD_NAMES[method]} "
                    f"episode={episode + 1}/{episode_count}",
                )

            for state_index, states in enumerate(states_by_episode[episode]):
                mass = covered_probability_mass(
                    points,
                    truth_density[episode],
                    weights,
                    states,
                    fov_degrees=float(fov_degrees[method]),
                    sensing_range=float(sensing_ranges[method]),
                )
                covered_mass[method, episode, state_index] = mass
                coverage[method, episode, state_index] = (
                    footprint_normalized_coverage(mass, maximum_mass)
                )
                completed_work += 1
                if progress_callback is not None:
                    progress_callback(
                        completed_work,
                        total_work,
                        f"coverage method={METHOD_NAMES[method]} "
                        f"episode={episode + 1}/{episode_count} "
                        f"state={state_index + 1}/{state_count}",
                    )

    duration = float(state_times[-1] - state_times[0])
    if duration <= 0.0:
        raise ValueError("state_times must span positive mission time")
    final_covered_mass = covered_mass[:, :, -1]
    final_coverage = coverage[:, :, -1]
    time_average_covered_mass = (
        np.trapezoid(covered_mass, state_times, axis=2) / duration
    )
    time_average_coverage = np.trapezoid(coverage, state_times, axis=2) / duration

    runtime_total = np.stack(
        (
            _total_step_timings(
                multifidelity_metadata,
                multifidelity,
                episode_count,
                state_count - 1,
            ),
            _total_step_timings(
                egerstedt_metadata,
                egerstedt,
                episode_count,
                state_count - 1,
            ),
        ),
        axis=0,
    )
    runtime_statistics = np.stack(
        (
            np.mean(runtime_total, axis=2),
            np.median(runtime_total, axis=2),
            np.percentile(runtime_total, 95.0, axis=2),
            np.max(runtime_total, axis=2),
        ),
        axis=-1,
    )

    metadata = {
        "multifidelity_source": str(Path(multifidelity_path)),
        "egerstedt_source": str(Path(egerstedt_path)),
        "scenario_fingerprint": fingerprint,
        "methods": METHOD_NAMES.tolist(),
        "episodes": int(episode_count),
        "num_steps": int(state_count - 1),
        "dt": float(multifidelity_metadata["dt"]),
        "timing_statistics": list(TIMING_STATISTICS),
        "comparable_metrics": {
            "kl": False,
            "nrmse": False,
            "calibration_95": False,
            "coverage": True,
            "runtime": True,
        },
        "computed_metrics": [
            "covered_probability_mass",
            "coverage",
            "runtime_total",
        ],
        "metric_definitions": {
            "covered_probability_mass": (
                "shared hidden-truth probability mass inside the union of each "
                "method's actual instantaneous ground sensing footprints"
            ),
            "coverage": (
                "covered_probability_mass divided by the hidden-truth mass in "
                "the densest free-space subset with area min(N * sector_area, "
                "free_space_area); each method uses its saved footprint geometry"
            ),
            "runtime_total": (
                "saved total wall-clock duration of one complete method step"
            ),
        },
    }
    arrays = {
        "method_names": METHOD_NAMES,
        "seeds": np.array(seeds, copy=True),
        "state_times": np.array(state_times, copy=True),
        "fov_degrees": fov_degrees,
        "sensing_ranges": sensing_ranges,
        "covered_probability_mass": covered_mass,
        "coverage": coverage,
        "coverage_oracle_mass": oracle_mass,
        "coverage_area_budget": area_budget,
        "coverage_free_area": free_area,
        "coverage_area_fraction": area_fraction,
        "final_covered_probability_mass": final_covered_mass,
        "final_coverage": final_coverage,
        "time_average_covered_probability_mass": time_average_covered_mass,
        "time_average_coverage": time_average_coverage,
        "runtime_total": runtime_total,
        "runtime_statistics": runtime_statistics,
    }
    return save_archive(
        output_path,
        kind=BASELINE_COMPARISON_EVALUATED_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_pair(
    multifidelity_metadata: dict[str, object],
    multifidelity: dict[str, np.ndarray],
    egerstedt_metadata: dict[str, object],
    egerstedt: dict[str, np.ndarray],
) -> str:
    multifidelity_fingerprint = scenario_fingerprint(multifidelity)
    egerstedt_fingerprint = scenario_fingerprint(egerstedt)
    recorded_multifidelity = multifidelity_metadata.get("scenario_fingerprint")
    recorded_egerstedt = egerstedt_metadata.get("scenario_fingerprint")
    if not (
        recorded_multifidelity
        == recorded_egerstedt
        == multifidelity_fingerprint
        == egerstedt_fingerprint
    ):
        raise ValueError("method archives do not contain the same paired scenario")
    for name in (
        "seeds",
        "state_times",
        "query_points",
        "integration_weights",
        "truth_density",
        "free_mask",
    ):
        if not np.array_equal(multifidelity[name], egerstedt[name]):
            raise ValueError(f"method archives differ in paired array {name!r}")
    times = multifidelity["state_times"]
    if times.ndim != 1 or times.size < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError("state_times must be a strictly increasing vector")
    return multifidelity_fingerprint


def _multifidelity_ground_states(
    archive: dict[str, np.ndarray], episode_count: int, state_count: int
) -> np.ndarray:
    states = np.asarray(archive["ground_states"], dtype=float)
    if (
        states.ndim != 4
        or states.shape[0] != episode_count
        or states.shape[1] != state_count
        or states.shape[2] < 1
        or states.shape[3] < 3
        or not np.all(np.isfinite(states))
    ):
        raise ValueError(
            "multifidelity ground_states must have shape "
            "(episodes, states, robots, state_dim>=3)"
        )
    return states


def _egerstedt_ground_states(
    archive: dict[str, np.ndarray], episode_count: int, state_count: int
) -> np.ndarray:
    positions = np.asarray(archive["ground_positions"], dtype=float)
    if (
        positions.ndim != 4
        or positions.shape[0] != episode_count
        or positions.shape[1] != state_count
        or positions.shape[2] < 1
        or positions.shape[3] != 2
        or not np.all(np.isfinite(positions))
    ):
        raise ValueError(
            "egerstedt ground_positions must have shape "
            "(episodes, states, robots, 2)"
        )
    headings = np.zeros((*positions.shape[:-1], 1), dtype=float)
    return np.concatenate((positions, headings), axis=-1)


def _total_step_timings(
    metadata: dict[str, object],
    archive: dict[str, np.ndarray],
    episode_count: int,
    step_count: int,
) -> np.ndarray:
    fields = metadata.get("timing_fields")
    if not isinstance(fields, list) or "total" not in fields:
        raise ValueError("archive metadata must list a total timing field")
    timings = np.asarray(archive["step_timings"], dtype=float)
    if (
        timings.ndim != 3
        or timings.shape[0] != episode_count
        or timings.shape[1] != step_count
        or timings.shape[2] != len(fields)
        or not np.all(np.isfinite(timings))
        or np.any(timings < 0.0)
    ):
        raise ValueError(
            "step_timings must have shape (episodes, steps, timing_fields)"
        )
    return timings[:, :, fields.index("total")]


def _scalar(value: np.ndarray, name: str) -> float:
    array = np.asarray(value, dtype=float)
    if array.shape != () or not np.isfinite(array.item()):
        raise ValueError(f"{name} must be a finite scalar")
    result = float(array.item())
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Comparison metrics")
    try:
        path = evaluate_baseline_comparison(
            arguments.multifidelity,
            arguments.egerstedt,
            arguments.output,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"evaluated_results={path}")
    if not arguments.no_plot:
        from evaluation.plot_baseline_comparison_metrics import (
            plot_baseline_comparison_metrics,
        )

        plot_output = arguments.plot_output
        if plot_output is None:
            plot_output = path.with_name("coverage_metrics.png")
        plot_path = plot_baseline_comparison_metrics(path, plot_output)
        print(f"baseline_metrics_plot={plot_path}")


if __name__ == "__main__":
    main()
