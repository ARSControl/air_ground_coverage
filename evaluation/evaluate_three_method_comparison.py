"""Evaluate paired multifidelity, Egerstedt, and Zhang raw archives."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    THREE_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
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


METHOD_NAMES = np.asarray(("multifidelity", "egerstedt", "zhang"))
TIMING_STATISTICS = ("mean", "median", "p95", "maximum")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/three_method_comparison")
    parser.add_argument(
        "--multifidelity", type=Path, default=root / "multifidelity/raw.npz"
    )
    parser.add_argument("--egerstedt", type=Path, default=root / "egerstedt/raw.npz")
    parser.add_argument("--zhang", type=Path, default=root / "zhang/raw.npz")
    parser.add_argument("--output", "-o", type=Path, default=root / "evaluated.npz")
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def evaluate_three_method_comparison(
    multifidelity_path: str | Path,
    egerstedt_path: str | Path,
    zhang_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 2000,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute common metrics without rerunning any controller."""
    if (
        isinstance(bootstrap_samples, bool)
        or not isinstance(bootstrap_samples, int)
        or bootstrap_samples < 1
    ):
        raise ValueError("bootstrap_samples must be a positive integer")
    multifidelity_metadata, multifidelity = load_archive(
        multifidelity_path,
        expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    )
    egerstedt_metadata, egerstedt = load_archive(
        egerstedt_path,
        expected_kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    )
    zhang_metadata, zhang = load_archive(
        zhang_path,
        expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    )
    fingerprint = _validate_archives(
        (multifidelity_metadata, egerstedt_metadata, zhang_metadata),
        (multifidelity, egerstedt, zhang),
    )
    seeds = multifidelity["seeds"]
    times = multifidelity["state_times"]
    episode_count = seeds.size
    state_count = times.size
    step_count = state_count - 1
    ground_states = (
        _states(multifidelity["ground_states"], episode_count, state_count, True),
        _states(egerstedt["ground_positions"], episode_count, state_count, False),
        _states(zhang["ground_positions"], episode_count, state_count, False),
    )
    positions = tuple(states[..., :2] for states in ground_states)
    fov_degrees = np.asarray(
        [
            _scalar(archive["ground_fov_degrees"], "ground_fov_degrees")
            for archive in (multifidelity, egerstedt, zhang)
        ]
    )
    sensing_ranges = np.asarray(
        [
            _scalar(archive["ground_sensing_range"], "ground_sensing_range")
            for archive in (multifidelity, egerstedt, zhang)
        ]
    )
    points = multifidelity["query_points"]
    weights = multifidelity["integration_weights"]
    truth = multifidelity["truth_density"]
    free_masks = multifidelity["free_mask"].astype(bool)
    shape = (METHOD_NAMES.size, episode_count, state_count)
    covered_mass = np.empty(shape, dtype=float)
    coverage = np.empty(shape, dtype=float)
    locational_cost = np.empty(shape, dtype=float)
    oracle_mass = np.empty((METHOD_NAMES.size, episode_count), dtype=float)
    area_budget = np.empty_like(oracle_mass)
    total_work = METHOD_NAMES.size * episode_count * state_count
    completed = 0
    if progress_callback is not None:
        progress_callback(0, total_work, "validated three paired raw archives")

    for method, method_states in enumerate(ground_states):
        for episode in range(episode_count):
            free_area = float(np.sum(weights[free_masks[episode]], dtype=float))
            nominal_area = method_states.shape[2] * sector_footprint_area(
                fov_degrees=float(fov_degrees[method]),
                sensing_range=float(sensing_ranges[method]),
            )
            budget = min(nominal_area, free_area)
            maximum_mass = maximum_density_mass_for_area(
                truth[episode],
                weights,
                area_budget=budget,
                free_mask=free_masks[episode],
            )
            area_budget[method, episode] = budget
            oracle_mass[method, episode] = maximum_mass
            for state_index, state in enumerate(method_states[episode]):
                mass = covered_probability_mass(
                    points,
                    truth[episode],
                    weights,
                    state,
                    fov_degrees=float(fov_degrees[method]),
                    sensing_range=float(sensing_ranges[method]),
                )
                covered_mass[method, episode, state_index] = mass
                coverage[method, episode, state_index] = footprint_normalized_coverage(
                    mass, maximum_mass
                )
                distances = np.sum(
                    (points[:, None, :] - state[None, :, :2]) ** 2, axis=2
                )
                locational_cost[method, episode, state_index] = float(
                    np.sum(
                        np.min(distances, axis=1) * truth[episode] * weights,
                        dtype=float,
                    )
                )
                completed += 1
                if progress_callback is not None:
                    progress_callback(
                        completed,
                        total_work,
                        f"method={METHOD_NAMES[method]} "
                        f"episode={episode + 1}/{episode_count} "
                        f"state={state_index + 1}/{state_count}",
                    )

    duration = float(times[-1] - times[0])
    final_coverage = coverage[:, :, -1]
    time_average_coverage = np.trapezoid(coverage, times, axis=2) / duration
    final_cost = locational_cost[:, :, -1]
    time_average_cost = np.trapezoid(locational_cost, times, axis=2) / duration
    ground_travel = np.stack([_cumulative_travel(value) for value in positions], axis=0)
    aerial_positions = (
        multifidelity["aerial_states"][..., :2],
        egerstedt["aerial_positions"],
        zhang["aerial_positions"],
    )
    aerial_travel = np.stack(
        [_cumulative_travel(value) for value in aerial_positions], axis=0
    )
    runtime_total = np.stack(
        [
            _total_timings(metadata, archive, episode_count, step_count)
            for metadata, archive in (
                (multifidelity_metadata, multifidelity),
                (egerstedt_metadata, egerstedt),
                (zhang_metadata, zhang),
            )
        ],
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
    rng = np.random.default_rng(20260808)
    final_coverage_ci = _bootstrap_mean_ci(final_coverage, bootstrap_samples, rng)
    time_average_coverage_ci = _bootstrap_mean_ci(
        time_average_coverage, bootstrap_samples, rng
    )
    final_cost_ci = _bootstrap_mean_ci(final_cost, bootstrap_samples, rng)
    time_average_cost_ci = _bootstrap_mean_ci(time_average_cost, bootstrap_samples, rng)
    paired_final_coverage_difference = final_coverage - final_coverage[0:1]
    paired_time_average_coverage_difference = (
        time_average_coverage - time_average_coverage[0:1]
    )
    paired_final_coverage_difference_ci = _bootstrap_mean_ci(
        paired_final_coverage_difference, bootstrap_samples, rng
    )
    paired_time_average_coverage_difference_ci = _bootstrap_mean_ci(
        paired_time_average_coverage_difference, bootstrap_samples, rng
    )

    metadata = {
        "methods": METHOD_NAMES.tolist(),
        "multifidelity_source": str(Path(multifidelity_path)),
        "egerstedt_source": str(Path(egerstedt_path)),
        "zhang_source": str(Path(zhang_path)),
        "scenario_fingerprint": fingerprint,
        "episodes": int(episode_count),
        "num_steps": int(step_count),
        "dt": float(multifidelity_metadata["dt"]),
        "bootstrap_samples": bootstrap_samples,
        "timing_statistics": list(TIMING_STATISTICS),
        "comparable_metrics": {
            "coverage": True,
            "locational_cost": True,
            "ground_travel_distance": True,
            "aerial_travel_distance": True,
            "runtime": True,
            "reconstruction": {
                "multifidelity": True,
                "egerstedt": False,
                "zhang": False,
            },
        },
        "zhang_mission_clock": zhang_metadata["mission_clock"],
        "zhang_variant": zhang_metadata.get(
            "aerial_sensing",
            {"mode": "full_cell", "paper_faithful": True},
        ),
    }
    arrays = {
        "method_names": METHOD_NAMES,
        "seeds": np.array(seeds, copy=True),
        "state_times": np.array(times, copy=True),
        "fov_degrees": fov_degrees,
        "sensing_ranges": sensing_ranges,
        "covered_probability_mass": covered_mass,
        "coverage": coverage,
        "coverage_oracle_mass": oracle_mass,
        "coverage_area_budget": area_budget,
        "final_coverage": final_coverage,
        "time_average_coverage": time_average_coverage,
        "final_coverage_ci": final_coverage_ci,
        "time_average_coverage_ci": time_average_coverage_ci,
        "paired_final_coverage_difference": paired_final_coverage_difference,
        "paired_time_average_coverage_difference": (
            paired_time_average_coverage_difference
        ),
        "paired_final_coverage_difference_ci": (paired_final_coverage_difference_ci),
        "paired_time_average_coverage_difference_ci": (
            paired_time_average_coverage_difference_ci
        ),
        "locational_cost": locational_cost,
        "final_locational_cost": final_cost,
        "time_average_locational_cost": time_average_cost,
        "final_locational_cost_ci": final_cost_ci,
        "time_average_locational_cost_ci": time_average_cost_ci,
        "ground_travel_distance": ground_travel,
        "aerial_travel_distance": aerial_travel,
        "runtime_total": runtime_total,
        "runtime_statistics": runtime_statistics,
        "zhang_phase_transition_time": np.array(
            zhang["phase_transition_time"], copy=True
        ),
        "zhang_mission_phase": np.array(zhang["mission_phase"], copy=True),
        "zhang_aerial_cell_support_fraction": np.array(
            zhang.get(
                "aerial_cell_support_fraction",
                np.ones(zhang["aerial_positions"].shape[:3], dtype=float),
            ),
            copy=True,
        ),
    }
    return save_archive(
        output_path,
        kind=THREE_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_archives(
    metadata: tuple[dict[str, object], ...],
    archives: tuple[dict[str, np.ndarray], ...],
) -> str:
    fingerprints = tuple(scenario_fingerprint(archive) for archive in archives)
    recorded = tuple(item.get("scenario_fingerprint") for item in metadata)
    if len(set((*fingerprints, *recorded))) != 1:
        raise ValueError("method archives do not contain the same paired scenario")
    for name in (
        "seeds",
        "state_times",
        "query_points",
        "integration_weights",
        "truth_density",
        "free_mask",
    ):
        if not all(
            np.array_equal(archives[0][name], value[name]) for value in archives[1:]
        ):
            raise ValueError(f"method archives differ in paired array {name!r}")
    return fingerprints[0]


def _states(
    values: np.ndarray, episode_count: int, state_count: int, has_heading: bool
) -> np.ndarray:
    states = np.asarray(values, dtype=float)
    required_dimension = 3 if has_heading else 2
    if (
        states.ndim != 4
        or states.shape[:2] != (episode_count, state_count)
        or states.shape[2] < 1
        or states.shape[3] < required_dimension
        or not np.all(np.isfinite(states))
    ):
        raise ValueError("ground state archive has an invalid shape or value")
    if has_heading:
        return states[..., :3]
    return np.concatenate(
        (states[..., :2], np.zeros((*states.shape[:-1], 1), dtype=float)), axis=-1
    )


def _cumulative_travel(positions: np.ndarray) -> np.ndarray:
    increments = np.sum(np.linalg.norm(np.diff(positions, axis=1), axis=-1), axis=2)
    return np.concatenate(
        (np.zeros((positions.shape[0], 1)), np.cumsum(increments, axis=1)), axis=1
    )


def _total_timings(
    metadata: dict[str, object],
    archive: dict[str, np.ndarray],
    episode_count: int,
    step_count: int,
) -> np.ndarray:
    fields = metadata.get("timing_fields")
    if not isinstance(fields, list) or "total" not in fields:
        raise ValueError("archive metadata must identify its total timing field")
    timings = np.asarray(archive["step_timings"], dtype=float)
    if timings.shape != (episode_count, step_count, len(fields)):
        raise ValueError("step_timings do not match archive metadata")
    result = timings[:, :, fields.index("total")]
    if np.any(~np.isfinite(result)) or np.any(result < 0.0):
        raise ValueError("total step timings must be finite and nonnegative")
    return result


def _bootstrap_mean_ci(
    values: np.ndarray, samples: int, rng: np.random.Generator
) -> np.ndarray:
    method_count, episode_count = values.shape
    indices = rng.integers(0, episode_count, size=(samples, episode_count))
    means = np.empty((method_count, samples), dtype=float)
    for method in range(method_count):
        means[method] = np.mean(values[method][indices], axis=1)
    return np.percentile(means, (2.5, 97.5), axis=1).T


def _scalar(value: np.ndarray, name: str) -> float:
    array = np.asarray(value, dtype=float)
    if array.size != 1 or not np.isfinite(array.reshape(-1)[0]):
        raise ValueError(f"{name} must be a finite scalar")
    return float(array.reshape(-1)[0])


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Three methods")
    try:
        path = evaluate_three_method_comparison(
            arguments.multifidelity,
            arguments.egerstedt,
            arguments.zhang,
            arguments.output,
            bootstrap_samples=arguments.bootstrap_samples,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"three_method_evaluated={path}")


if __name__ == "__main__":
    main()
