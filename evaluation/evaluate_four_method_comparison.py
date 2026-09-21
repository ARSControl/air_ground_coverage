"""Evaluate the paired limited/full-range four-condition comparison."""

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
    FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    SCENARIO_ARRAYS,
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


METHOD_NAMES = np.asarray(
    ("multifidelity_lr", "egerstedt_lr", "egerstedt_full", "zhang_full")
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/four_method_comparison")
    parser.add_argument(
        "--multifidelity", type=Path, default=root / "multifidelity/raw.npz"
    )
    parser.add_argument(
        "--egerstedt-lr", type=Path, default=root / "egerstedt_lr/raw.npz"
    )
    parser.add_argument(
        "--egerstedt-full", type=Path, default=root / "egerstedt_full/raw.npz"
    )
    parser.add_argument("--zhang", type=Path, default=root / "zhang/raw.npz")
    parser.add_argument("--output", "-o", type=Path, default=root / "evaluated.npz")
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def evaluate_four_method_comparison(
    multifidelity_path: str | Path,
    egerstedt_lr_path: str | Path,
    egerstedt_full_path: str | Path,
    zhang_path: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int = 2000,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute only the common coverage and locational-cost evidence."""
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
    egerstedt_lr_metadata, egerstedt_lr = load_archive(
        egerstedt_lr_path,
        expected_kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    )
    egerstedt_full_metadata, egerstedt_full = load_archive(
        egerstedt_full_path,
        expected_kind=EGERSTEDT_COMPARISON_RAW_ARCHIVE_KIND,
    )
    zhang_metadata, zhang = load_archive(
        zhang_path,
        expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    )
    metadata_items = (
        multifidelity_metadata,
        egerstedt_lr_metadata,
        egerstedt_full_metadata,
        zhang_metadata,
    )
    archives = (multifidelity, egerstedt_lr, egerstedt_full, zhang)
    fingerprint = _validate_archives(metadata_items, archives)
    episodes = multifidelity["seeds"].size
    times = multifidelity["state_times"]
    states = times.size
    ground_states = (
        _states(multifidelity["ground_states"], episodes, states, has_heading=True),
        _states(egerstedt_lr["ground_positions"], episodes, states),
        _states(egerstedt_full["ground_positions"], episodes, states),
        _states(zhang["ground_positions"], episodes, states),
    )
    ground_fov = np.asarray(
        [_scalar(values["ground_fov_degrees"]) for values in archives]
    )
    ground_range = np.asarray(
        [_scalar(values["ground_sensing_range"]) for values in archives]
    )
    points = multifidelity["query_points"]
    weights = multifidelity["integration_weights"]
    truth = multifidelity["truth_density"]
    free = multifidelity["free_mask"].astype(bool)
    shape = (METHOD_NAMES.size, episodes, states)
    covered_mass = np.empty(shape, dtype=float)
    coverage = np.empty(shape, dtype=float)
    locational_cost = np.empty(shape, dtype=float)
    oracle_mass = np.empty((METHOD_NAMES.size, episodes), dtype=float)
    total = METHOD_NAMES.size * episodes * states
    completed = 0
    if progress_callback is not None:
        progress_callback(0, total, "validated four exactly paired conditions")

    for method, method_states in enumerate(ground_states):
        for episode in range(episodes):
            free_area = float(np.sum(weights[free[episode]], dtype=float))
            nominal_area = method_states.shape[2] * sector_footprint_area(
                fov_degrees=float(ground_fov[method]),
                sensing_range=float(ground_range[method]),
            )
            maximum = maximum_density_mass_for_area(
                truth[episode],
                weights,
                area_budget=min(nominal_area, free_area),
                free_mask=free[episode],
            )
            oracle_mass[method, episode] = maximum
            for state, state_values in enumerate(method_states[episode]):
                mass = covered_probability_mass(
                    points,
                    truth[episode],
                    weights,
                    state_values,
                    fov_degrees=float(ground_fov[method]),
                    sensing_range=float(ground_range[method]),
                )
                covered_mass[method, episode, state] = mass
                coverage[method, episode, state] = footprint_normalized_coverage(
                    mass, maximum
                )
                distances = np.sum(
                    (points[:, None, :] - state_values[None, :, :2]) ** 2, axis=2
                )
                locational_cost[method, episode, state] = float(
                    np.sum(
                        np.min(distances, axis=1) * truth[episode] * weights,
                        dtype=float,
                    )
                )
                completed += 1
                if progress_callback is not None:
                    progress_callback(
                        completed,
                        total,
                        f"method={METHOD_NAMES[method]} "
                        f"episode={episode + 1}/{episodes} state={state + 1}/{states}",
                    )

    duration = float(times[-1] - times[0])
    final_coverage = coverage[:, :, -1]
    average_coverage = np.trapezoid(coverage, times, axis=2) / duration
    final_cost = locational_cost[:, :, -1]
    average_cost = np.trapezoid(locational_cost, times, axis=2) / duration
    rng = np.random.default_rng(20260809)
    bootstrap_indices = rng.integers(0, episodes, size=(bootstrap_samples, episodes))
    effective_ranges = np.asarray(
        (
            _scalar(multifidelity["aerial_sensing_range"]),
            _scalar(egerstedt_lr["effective_aerial_sensing_range"]),
            _scalar(egerstedt_full["effective_aerial_sensing_range"]),
            np.inf,
        )
    )
    metadata = {
        "methods": METHOD_NAMES.tolist(),
        "scenario_fingerprint": fingerprint,
        "episodes": int(episodes),
        "num_steps": int(states - 1),
        "dt": float(multifidelity_metadata["dt"]),
        "bootstrap_samples": bootstrap_samples,
        "paired_bootstrap_seed": 20260809,
        "final_plot_metrics": [
            "footprint_normalized_coverage",
            "locational_cost",
        ],
        "information_regimes": {
            "multifidelity_lr": "limited aerial observations",
            "egerstedt_lr": "limited aerial partition; oracle truth density",
            "egerstedt_full": "full-domain aerial partition; oracle truth density",
            "zhang_full": "complete LOW-field power cells",
        },
        "egerstedt_lr_aerial_sensing": egerstedt_lr_metadata["aerial_sensing"],
        "egerstedt_full_aerial_sensing": egerstedt_full_metadata["aerial_sensing"],
        "zhang_variant": zhang_metadata["aerial_sensing"],
        "metric_definitions": {
            "coverage": "ground-footprint-normalized hidden-truth probability mass",
            "locational_cost": "truth-weighted squared distance to nearest ground robot",
        },
    }
    arrays = {
        "method_names": METHOD_NAMES,
        "seeds": np.array(multifidelity["seeds"], copy=True),
        "state_times": np.array(times, copy=True),
        "effective_aerial_sensing_ranges": effective_ranges,
        "ground_fov_degrees": ground_fov,
        "ground_sensing_ranges": ground_range,
        "covered_probability_mass": covered_mass,
        "coverage_oracle_mass": oracle_mass,
        "coverage": coverage,
        "coverage_history_ci": _bootstrap_mean_ci(coverage, bootstrap_indices),
        "final_coverage": final_coverage,
        "final_coverage_ci": _bootstrap_mean_ci(final_coverage, bootstrap_indices),
        "time_average_coverage": average_coverage,
        "time_average_coverage_ci": _bootstrap_mean_ci(
            average_coverage, bootstrap_indices
        ),
        "locational_cost": locational_cost,
        "locational_cost_history_ci": _bootstrap_mean_ci(
            locational_cost, bootstrap_indices
        ),
        "final_locational_cost": final_cost,
        "final_locational_cost_ci": _bootstrap_mean_ci(final_cost, bootstrap_indices),
        "time_average_locational_cost": average_cost,
        "time_average_locational_cost_ci": _bootstrap_mean_ci(
            average_cost, bootstrap_indices
        ),
    }
    return save_archive(
        output_path,
        kind=FOUR_METHOD_COMPARISON_EVALUATED_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_archives(
    metadata: tuple[dict[str, object], ...],
    archives: tuple[dict[str, np.ndarray], ...],
) -> str:
    fingerprints = tuple(scenario_fingerprint(values) for values in archives)
    recorded = tuple(item.get("scenario_fingerprint") for item in metadata)
    if len(set((*fingerprints, *recorded))) != 1:
        raise ValueError("all four conditions must share one scenario fingerprint")
    for name in SCENARIO_ARRAYS:
        if not all(
            np.array_equal(archives[0][name], values[name]) for values in archives[1:]
        ):
            raise ValueError(f"conditions differ in scenario array {name!r}")
    if (
        metadata[1].get("method") != "egerstedt"
        or metadata[2].get("method") != "egerstedt"
    ):
        raise ValueError("middle conditions must be Egerstedt variants")
    limited = metadata[1].get("aerial_sensing")
    full = metadata[2].get("aerial_sensing")
    if not isinstance(limited, dict) or limited.get("overridden") is not False:
        raise ValueError("Egerstedt-LR must use the scenario aerial range")
    if not isinstance(full, dict) or full.get("covers_complete_domain") is not True:
        raise ValueError("full-range Egerstedt must cover the complete domain")
    zhang = metadata[3].get("aerial_sensing")
    if not isinstance(zhang, dict) or zhang.get("mode") != "full_cell":
        raise ValueError("the fourth condition must be standard full-cell Zhang")
    return fingerprints[0]


def _states(
    values: np.ndarray,
    episodes: int,
    states: int,
    *,
    has_heading: bool = False,
) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    required = 3 if has_heading else 2
    if (
        array.ndim != 4
        or array.shape[:2] != (episodes, states)
        or array.shape[2] < 1
        or array.shape[3] < required
        or not np.all(np.isfinite(array))
    ):
        raise ValueError("ground-state archive has an invalid shape or value")
    if has_heading:
        return array[..., :3]
    return np.concatenate(
        (array[..., :2], np.zeros((*array.shape[:-1], 1), dtype=float)), axis=-1
    )


def _bootstrap_mean_ci(values: np.ndarray, indices: np.ndarray) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    methods, episodes, *tail = array.shape
    if indices.ndim != 2 or indices.shape[1] != episodes:
        raise ValueError("bootstrap indices do not match the episode axis")
    means = np.empty((methods, indices.shape[0], *tail), dtype=float)
    chunk_size = 128
    for method in range(methods):
        for start in range(0, indices.shape[0], chunk_size):
            stop = min(start + chunk_size, indices.shape[0])
            means[method, start:stop] = np.mean(
                array[method][indices[start:stop]], axis=1
            )
    bounds = np.percentile(means, (2.5, 97.5), axis=1)
    return np.moveaxis(bounds, 0, -1)


def _scalar(value: np.ndarray) -> float:
    array = np.asarray(value, dtype=float)
    if array.size != 1 or not np.isfinite(array.reshape(-1)[0]):
        raise ValueError("expected a finite scalar archive value")
    return float(array.reshape(-1)[0])


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Four conditions")
    try:
        output = evaluate_four_method_comparison(
            arguments.multifidelity,
            arguments.egerstedt_lr,
            arguments.egerstedt_full,
            arguments.zhang,
            arguments.output,
            bootstrap_samples=arguments.bootstrap_samples,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"four_method_evaluated={output}")


if __name__ == "__main__":
    main()
