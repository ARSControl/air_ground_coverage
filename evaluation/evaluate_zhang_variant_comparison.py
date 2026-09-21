"""Evaluate standard Zhang and Zhang-LR on one exact paired scenario."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    SCENARIO_ARRAYS,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    ZHANG_VARIANT_COMPARISON_EVALUATED_ARCHIVE_KIND,
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


METHOD_NAMES = np.asarray(("zhang", "zhang_lr"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path("output/zhang_variant_comparison")
    parser.add_argument("--standard", type=Path, default=root / "standard/raw.npz")
    parser.add_argument("--lr", type=Path, default=root / "lr/raw.npz")
    parser.add_argument("--output", "-o", type=Path, default=root / "evaluated.npz")
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def evaluate_zhang_variant_comparison(
    standard_path: str | Path,
    lr_path: str | Path,
    output_path: str | Path,
    *,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute direct paired differences without rerunning either controller."""
    standard_metadata, standard = load_archive(
        standard_path, expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND
    )
    lr_metadata, limited = load_archive(
        lr_path, expected_kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND
    )
    fingerprint = _validate_pair(standard_metadata, standard, lr_metadata, limited)
    episodes = standard["seeds"].size
    state_times = standard["state_times"]
    states = state_times.size
    points = standard["query_points"]
    weights = standard["integration_weights"]
    truth = standard["truth_density"]
    free = standard["free_mask"].astype(bool)
    ground = np.stack(
        (standard["ground_positions"], limited["ground_positions"]), axis=0
    )
    aerial = np.stack(
        (standard["aerial_positions"], limited["aerial_positions"]), axis=0
    )
    ground_fov = np.asarray(
        (
            _scalar(standard["ground_fov_degrees"]),
            _scalar(limited["ground_fov_degrees"]),
        )
    )
    ground_range = np.asarray(
        (
            _scalar(standard["ground_sensing_range"]),
            _scalar(limited["ground_sensing_range"]),
        )
    )
    coverage = np.empty((2, episodes, states), dtype=float)
    covered_mass = np.empty_like(coverage)
    locational_cost = np.empty_like(coverage)
    oracle_mass = np.empty((2, episodes), dtype=float)
    total = 2 * episodes * states
    completed = 0
    if progress_callback is not None:
        progress_callback(0, total, "validated paired Zhang variants")

    for method in range(2):
        for episode in range(episodes):
            nominal_area = ground.shape[3] * sector_footprint_area(
                fov_degrees=float(ground_fov[method]),
                sensing_range=float(ground_range[method]),
            )
            free_area = float(np.sum(weights[free[episode]], dtype=float))
            maximum = maximum_density_mass_for_area(
                truth[episode],
                weights,
                area_budget=min(nominal_area, free_area),
                free_mask=free[episode],
            )
            oracle_mass[method, episode] = maximum
            for state in range(states):
                ground_state = np.column_stack(
                    (
                        ground[method, episode, state],
                        np.zeros(ground.shape[3], dtype=float),
                    )
                )
                mass = covered_probability_mass(
                    points,
                    truth[episode],
                    weights,
                    ground_state,
                    fov_degrees=float(ground_fov[method]),
                    sensing_range=float(ground_range[method]),
                )
                covered_mass[method, episode, state] = mass
                coverage[method, episode, state] = footprint_normalized_coverage(
                    mass, maximum
                )
                distances = np.sum(
                    (points[:, None, :] - ground[method, episode, state][None, :, :])
                    ** 2,
                    axis=2,
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

    aerial_difference = np.linalg.norm(aerial[0] - aerial[1], axis=-1)
    ground_difference = np.linalg.norm(ground[0] - ground[1], axis=-1)
    duration = float(state_times[-1] - state_times[0])
    support = np.stack(
        (
            standard.get(
                "aerial_cell_support_fraction",
                np.ones(standard["aerial_positions"].shape[:3], dtype=float),
            ),
            limited["aerial_cell_support_fraction"],
        ),
        axis=0,
    )
    first_aerial = _first_divergence_times(aerial_difference, state_times)
    first_ground = _first_divergence_times(ground_difference, state_times)
    metadata = {
        "methods": METHOD_NAMES.tolist(),
        "paired_scenario": True,
        "scenario_fingerprint": fingerprint,
        "standard_source": str(Path(standard_path)),
        "lr_source": str(Path(lr_path)),
        "episodes": int(episodes),
        "num_steps": int(states - 1),
        "dt": float(standard_metadata["dt"]),
        "standard_aerial_sensing": standard_metadata.get(
            "aerial_sensing", {"mode": "full_cell", "paper_faithful": True}
        ),
        "lr_aerial_sensing": lr_metadata["aerial_sensing"],
        "metric_definitions": {
            "position_difference": "per-robot Euclidean distance between paired states",
            "cell_support_fraction": "quadrature area of power-cell/disk intersection divided by complete cell area",
            "coverage": "ground-footprint-normalized hidden-truth probability mass",
            "locational_cost": "quadrature truth-weighted squared distance to nearest ground robot",
        },
    }
    arrays = {
        "method_names": METHOD_NAMES,
        "seeds": np.array(standard["seeds"], copy=True),
        "state_times": np.array(state_times, copy=True),
        "aerial_position_difference": aerial_difference,
        "ground_position_difference": ground_difference,
        "first_aerial_divergence_time": first_aerial,
        "first_ground_divergence_time": first_ground,
        "cell_support_fraction": support,
        "cell_masses": np.stack(
            (standard["cell_masses"], limited["cell_masses"]), axis=0
        ),
        "target_masses": np.stack(
            (standard["target_masses"], limited["target_masses"]), axis=0
        ),
        "target_loads": np.stack(
            (standard["target_loads"], limited["target_loads"]), axis=0
        ),
        "coverage": coverage,
        "covered_probability_mass": covered_mass,
        "coverage_oracle_mass": oracle_mass,
        "final_coverage": coverage[:, :, -1],
        "time_average_coverage": (
            np.trapezoid(coverage, state_times, axis=2) / duration
        ),
        "locational_cost": locational_cost,
        "final_locational_cost": locational_cost[:, :, -1],
        "time_average_locational_cost": (
            np.trapezoid(locational_cost, state_times, axis=2) / duration
        ),
        "ground_travel_distance": np.stack(
            (_cumulative_travel(ground[0]), _cumulative_travel(ground[1])), axis=0
        ),
        "aerial_travel_distance": np.stack(
            (_cumulative_travel(aerial[0]), _cumulative_travel(aerial[1])), axis=0
        ),
        "phase_transition_time": np.stack(
            (
                standard["phase_transition_time"],
                limited["phase_transition_time"],
            ),
            axis=0,
        ),
    }
    return save_archive(
        output_path,
        kind=ZHANG_VARIANT_COMPARISON_EVALUATED_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_pair(
    standard_metadata: dict[str, object],
    standard: dict[str, np.ndarray],
    lr_metadata: dict[str, object],
    limited: dict[str, np.ndarray],
) -> str:
    if standard_metadata.get("method") != "zhang":
        raise ValueError("standard archive must contain the full-cell Zhang method")
    if lr_metadata.get("method") != "zhang_lr":
        raise ValueError("LR archive must contain Zhang-LR")
    fingerprints = (
        scenario_fingerprint(standard),
        scenario_fingerprint(limited),
        standard_metadata.get("scenario_fingerprint"),
        lr_metadata.get("scenario_fingerprint"),
    )
    if len(set(fingerprints)) != 1:
        raise ValueError("Zhang variants must use one exact paired scenario")
    for name in SCENARIO_ARRAYS:
        if not np.array_equal(standard[name], limited[name]):
            raise ValueError(f"Zhang variants differ in scenario array {name!r}")
    for name in ("aerial_positions", "ground_positions", "cell_masses"):
        if standard[name].shape != limited[name].shape:
            raise ValueError(f"Zhang variants differ in result shape {name!r}")
    return fingerprints[0]


def _first_divergence_times(
    differences: np.ndarray, state_times: np.ndarray, tolerance: float = 1.0e-10
) -> np.ndarray:
    result = np.full(differences.shape[0], np.nan, dtype=float)
    for episode in range(differences.shape[0]):
        differing = np.flatnonzero(np.any(differences[episode] > tolerance, axis=1))
        if differing.size:
            result[episode] = state_times[differing[0]]
    return result


def _cumulative_travel(positions: np.ndarray) -> np.ndarray:
    increments = np.sum(np.linalg.norm(np.diff(positions, axis=1), axis=-1), axis=2)
    return np.concatenate(
        (np.zeros((positions.shape[0], 1)), np.cumsum(increments, axis=1)), axis=1
    )


def _scalar(value: np.ndarray) -> float:
    array = np.asarray(value, dtype=float)
    if array.size != 1 or not np.isfinite(array.reshape(-1)[0]):
        raise ValueError("expected a finite scalar archive value")
    return float(array.reshape(-1)[0])


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Zhang variants")
    try:
        output = evaluate_zhang_variant_comparison(
            arguments.standard,
            arguments.lr,
            arguments.output,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"zhang_variant_evaluated={output}")


if __name__ == "__main__":
    main()
