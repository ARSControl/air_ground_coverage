"""Run Zhang et al. on an exact saved proposed-method comparison scenario."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from time import perf_counter
from typing import Any, Callable

import numpy as np
from scipy.interpolate import RegularGridInterpolator
import yaml

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.comparison_io import (  # noqa: E402
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    SCENARIO_ARRAYS,
    ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
    scenario_fingerprint,
)
from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.zhang import ZhangParameters, ZhangSimulation  # noqa: E402


DEFAULT_SCENARIO = Path("output/three_method_comparison/multifidelity/raw.npz")
DEFAULT_CONFIG = Path("configs/zhang_comparison.yaml")
DEFAULT_OUTPUT = Path("output/three_method_comparison/zhang/raw.npz")
TIMING_FIELDS = ("controller", "consensus", "total")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", "-i", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", "-o", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_zhang_comparison(
    scenario_path: str | Path,
    output_path: str | Path,
    *,
    config_path: str | Path = DEFAULT_CONFIG,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run the clocked Zhang mission while reusing every paired scenario input."""
    source_metadata, source = load_archive(
        scenario_path,
        expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    )
    fingerprint = scenario_fingerprint(source)
    if source_metadata.get("scenario_fingerprint") != fingerprint:
        raise ValueError("scenario fingerprint does not match source archive metadata")
    rows, columns, x_bounds, y_bounds = _validate_source(source, source_metadata)
    document = _load_document(config_path)
    normalized_size = _positive_real(
        document.get("normalized_domain_size"), "normalized_domain_size"
    )
    algorithm = document.get("algorithm")
    if not isinstance(algorithm, dict):
        raise ValueError("algorithm must be a mapping")
    aerial_sensing_mode = _aerial_sensing_mode(
        algorithm.get("aerial_sensing_mode", "full_cell")
    )

    dt = float(source_metadata["dt"])
    parameters_base = {
        "dt": dt,
        "deployment_gain": _positive_real(
            algorithm.get("deployment_gain"), "algorithm.deployment_gain"
        ),
        "deployment_tolerance": _positive_real(
            algorithm.get("deployment_tolerance"),
            "algorithm.deployment_tolerance",
        ),
        "deployment_max_steps": _positive_integer(
            algorithm.get("deployment_max_steps"),
            "algorithm.deployment_max_steps",
        ),
        "consensus_gain": _positive_real(
            algorithm.get("consensus_gain"), "algorithm.consensus_gain"
        ),
        "consensus_gain_fraction": _optional_unit_fraction(
            algorithm.get("consensus_gain_fraction"),
            "algorithm.consensus_gain_fraction",
        ),
        "consensus_iterations": _positive_integer(
            algorithm.get("consensus_iterations"),
            "algorithm.consensus_iterations",
        ),
        "weight_gain": _positive_real(
            algorithm.get("weight_gain"), "algorithm.weight_gain"
        ),
        "aerial_gain": _positive_real(
            algorithm.get("aerial_gain"), "algorithm.aerial_gain"
        ),
        "transfer_gain": _positive_real(
            algorithm.get("transfer_gain"), "algorithm.transfer_gain"
        ),
        "ground_gain": _positive_real(
            algorithm.get("ground_gain"), "algorithm.ground_gain"
        ),
    }
    scale_x = normalized_size / (x_bounds[1] - x_bounds[0])
    scale_y = normalized_size / (y_bounds[1] - y_bounds[0])
    if not np.isclose(scale_x, scale_y):
        raise ValueError("paired Zhang comparison requires a square physical domain")
    scale = float(scale_x)
    offset = np.asarray((-normalized_size / 2.0, -normalized_size / 2.0))
    physical_origin = np.asarray((x_bounds[0], y_bounds[0]))

    def normalize(position: np.ndarray) -> np.ndarray:
        return (np.asarray(position, dtype=float) - physical_origin) * scale + offset

    def physical(position: np.ndarray) -> np.ndarray:
        return (np.asarray(position, dtype=float) - offset) / scale + physical_origin

    query_points = source["query_points"]
    normalized_points = normalize(query_points)
    normalized_weights = source["integration_weights"] * scale * scale
    episode_count = source["seeds"].size
    state_count = source["state_times"].size
    step_count = state_count - 1
    aerial_count = source["initial_aerial_states"].shape[1]
    normalized_ground_range = float(source["ground_sensing_range"]) * scale
    physical_aerial_range = float(source["aerial_sensing_range"])
    normalized_aerial_range = physical_aerial_range * scale
    parameters_base["aerial_sensing_range"] = (
        normalized_aerial_range if aerial_sensing_mode == "limited" else None
    )
    total_steps = episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "loading paired Zhang scenarios")

    aerial_runs: list[np.ndarray] = []
    ground_runs: list[np.ndarray] = []
    phase_runs: list[np.ndarray] = []
    weight_runs: list[np.ndarray] = []
    mass_runs: list[np.ndarray] = []
    load_runs: list[np.ndarray] = []
    support_runs: list[np.ndarray] = []
    target_load_runs: list[np.ndarray] = []
    target_mass_runs: list[np.ndarray] = []
    timing_runs: list[np.ndarray] = []
    transition_times: list[float] = []
    deployment_steps: list[int] = []
    consensus_gains: list[float] = []
    consensus_durations: list[float] = []

    for episode in range(episode_count):
        low_field = _resample_grid(
            source["low_truth_field_grid"][episode], query_points
        )
        high_field = np.maximum(source["high_truth"][episode], 0.0)
        simulation = ZhangSimulation(
            points=normalized_points,
            integration_weights=normalized_weights,
            grid_shape=(rows, columns),
            low_field=np.maximum(low_field, 0.0),
            high_field=high_field,
            aerial_positions=normalize(source["initial_aerial_states"][episode, :, :2]),
            ground_positions=normalize(source["initial_ground_states"][episode, :, :2]),
            parameters=ZhangParameters(
                **parameters_base,
                ground_sensing_range=normalized_ground_range,
            ),
            rng=np.random.default_rng(int(source["seeds"][episode]) + 60000),
            deployment_within_mission=True,
        )
        aerial_history = [
            np.array(source["initial_aerial_states"][episode, :, :2], copy=True)
        ]
        ground_history = [
            np.array(source["initial_ground_states"][episode, :, :2], copy=True)
        ]
        phase_history = [0]
        weight_history = [simulation.cell_weights.copy()]
        mass_history = [simulation.cell_masses.copy()]
        load_history = [simulation.cell_loads.copy()]
        support_history = [simulation.aerial_support_fractions.copy()]
        timings: list[tuple[float, float, float]] = []
        prior_consensus_duration = 0.0

        for step in range(step_count):
            started = perf_counter()
            result = simulation.step()
            total_duration = perf_counter() - started
            consensus_duration = (
                simulation.total_consensus_duration - prior_consensus_duration
            )
            prior_consensus_duration = simulation.total_consensus_duration
            controller_duration = max(0.0, total_duration - consensus_duration)
            timings.append((controller_duration, consensus_duration, total_duration))
            aerial_history.append(physical(simulation.aerial_positions))
            ground_history.append(physical(simulation.ground_positions))
            phase_history.append(0 if simulation.phase == "deployment" else 1)
            weight_history.append(simulation.cell_weights.copy())
            mass_history.append(result.cell_masses.copy())
            load_history.append(result.cell_loads.copy())
            support_history.append(result.aerial_support_fractions.copy())
            completed_steps += 1
            if progress_callback is not None:
                progress_callback(
                    completed_steps,
                    total_steps,
                    f"episode={episode + 1}/{episode_count} "
                    f"step={step + 1}/{step_count} phase={simulation.phase}",
                )

        aerial_runs.append(np.asarray(aerial_history, dtype=float))
        ground_runs.append(np.asarray(ground_history, dtype=float))
        phase_runs.append(np.asarray(phase_history, dtype=np.int8))
        weight_runs.append(np.asarray(weight_history, dtype=float))
        mass_runs.append(np.asarray(mass_history, dtype=float))
        load_runs.append(np.asarray(load_history, dtype=np.int64))
        support_runs.append(np.asarray(support_history, dtype=float))
        if simulation.allocation is None:
            target_load_runs.append(np.full(aerial_count, -1, dtype=np.int64))
            target_mass_runs.append(np.full(aerial_count, np.nan, dtype=float))
            consensus_gains.append(float("nan"))
        else:
            target_load_runs.append(simulation.target_loads.copy())
            target_mass_runs.append(simulation.target_masses.copy())
            consensus_gains.append(float(simulation.consensus_gain_used))
        timing_runs.append(np.asarray(timings, dtype=float))
        transition_times.append(
            float("nan")
            if simulation.phase_transition_time is None
            else simulation.phase_transition_time
        )
        deployment_steps.append(simulation.deployment_steps)
        consensus_durations.append(simulation.total_consensus_duration)

    method_name = "zhang_lr" if aerial_sensing_mode == "limited" else "zhang"
    metadata = {
        "method": method_name,
        "scenario_source": str(Path(scenario_path)),
        "scenario_fingerprint": fingerprint,
        "config_path": str(Path(config_path)),
        "episodes": int(episode_count),
        "num_steps": int(step_count),
        "dt": dt,
        "timing_fields": list(TIMING_FIELDS),
        "mission_clock": {
            "deployment_consumes_physical_time": True,
            "ground_stationary_during_deployment": True,
            "consensus_consumes_physical_time": False,
            "consensus_included_in_wall_runtime": True,
            "algorithm_2_receives_only_remaining_horizon": True,
        },
        "aerial_sensing": {
            "mode": aerial_sensing_mode,
            "paper_faithful": aerial_sensing_mode == "full_cell",
            "physical_range": physical_aerial_range,
            "normalized_range": normalized_aerial_range,
            "limited_region": (
                "power_cell_intersection_sensing_disk"
                if aerial_sensing_mode == "limited"
                else "complete_power_cell"
            ),
        },
        "coordinate_normalization": {
            "normalized_domain_size": normalized_size,
            "physical_x_bounds": list(x_bounds),
            "physical_y_bounds": list(y_bounds),
            "isotropic_scale": scale,
        },
        "algorithm_assumptions": {
            "convex_obstacle_free_domain": True,
            "kinematics": "first-order point robots",
            "communication": "logical power-cell neighbors in one Python process",
            "aerial_sensing_assumption": (
                "finite-range Zhang-LR extension"
                if aerial_sensing_mode == "limited"
                else "paper assumption: sufficiently large aerial sensing radius"
            ),
        },
        "resolved_config": document,
    }
    arrays = {name: np.array(source[name], copy=True) for name in SCENARIO_ARRAYS}
    arrays.update(
        {
            "method_name": np.asarray(method_name),
            "low_truth_field_grid": np.array(source["low_truth_field_grid"], copy=True),
            "high_truth": np.array(source["high_truth"], copy=True),
            "initial_aerial_positions": np.array(
                source["initial_aerial_states"][:, :, :2], copy=True
            ),
            "initial_ground_positions": np.array(
                source["initial_ground_states"][:, :, :2], copy=True
            ),
            "aerial_positions": np.asarray(aerial_runs, dtype=float),
            "ground_positions": np.asarray(ground_runs, dtype=float),
            "mission_phase": np.asarray(phase_runs, dtype=np.int8),
            "cell_weights": np.asarray(weight_runs, dtype=float),
            "cell_masses": np.asarray(mass_runs, dtype=float),
            "cell_loads": np.asarray(load_runs, dtype=np.int64),
            "aerial_cell_support_fraction": np.asarray(support_runs, dtype=float),
            "target_loads": np.asarray(target_load_runs, dtype=np.int64),
            "target_masses": np.asarray(target_mass_runs, dtype=float),
            "phase_transition_time": np.asarray(transition_times, dtype=float),
            "deployment_steps": np.asarray(deployment_steps, dtype=np.int64),
            "consensus_gain_used": np.asarray(consensus_gains, dtype=float),
            "consensus_wall_duration": np.asarray(consensus_durations, dtype=float),
            "aerial_fov_degrees": np.asarray(360.0),
            "ground_fov_degrees": np.asarray(360.0),
            "step_timings": np.asarray(timing_runs, dtype=float),
        }
    )
    if scenario_fingerprint(arrays) != fingerprint:
        raise RuntimeError("saved Zhang scenario inputs changed unexpectedly")
    return save_archive(
        output_path,
        kind=ZHANG_COMPARISON_RAW_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _validate_source(
    arrays: dict[str, np.ndarray], metadata: dict[str, object]
) -> tuple[int, int, tuple[float, float], tuple[float, float]]:
    maps = arrays["map_grid"]
    if np.any(maps != 0):
        raise ValueError(
            "paired Zhang comparison supports only obstacle-free scenarios"
        )
    aerial_count = arrays["initial_aerial_states"].shape[1]
    ground_count = arrays["initial_ground_states"].shape[1]
    if aerial_count <= 1 or ground_count <= aerial_count:
        raise ValueError("Zhang comparison requires aerial > 1 and ground > aerial")
    times = arrays["state_times"]
    if int(metadata["num_steps"]) != times.size - 1:
        raise ValueError("state_times must match source num_steps")
    dt = float(metadata["dt"])
    if not np.isclose(times[0], 0.0) or not np.allclose(np.diff(times), dt):
        raise ValueError("state_times must start at zero and use the source dt")
    points = np.asarray(arrays["query_points"], dtype=float)
    x_values = np.unique(points[:, 0])
    y_values = np.unique(points[:, 1])
    if x_values.size * y_values.size != points.shape[0]:
        raise ValueError("paired Zhang comparison requires a Cartesian grid")
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    if not np.array_equal(points, np.column_stack((grid_x.ravel(), grid_y.ravel()))):
        raise ValueError("query_points must use row-major Cartesian ordering")
    return (
        y_values.size,
        x_values.size,
        (float(x_values[0]), float(x_values[-1])),
        (float(y_values[0]), float(y_values[-1])),
    )


def _resample_grid(grid: np.ndarray, query_points: np.ndarray) -> np.ndarray:
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
    interpolator = RegularGridInterpolator((y_axis, x_axis), values)
    return np.asarray(interpolator(query_points[:, [1, 0]]), dtype=float)


def _load_document(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("Zhang comparison configuration must be a mapping")
    return document


def _positive_real(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _optional_unit_fraction(value: Any, name: str) -> float | None:
    if value is None:
        return None
    result = _positive_real(value, name)
    if result >= 1.0:
        raise ValueError(f"{name} must be strictly less than one")
    return result


def _aerial_sensing_mode(value: Any) -> str:
    if not isinstance(value, str):
        raise TypeError("algorithm.aerial_sensing_mode must be a string")
    mode = value.strip().lower()
    if mode not in {"full_cell", "limited"}:
        raise ValueError(
            "algorithm.aerial_sensing_mode must be 'full_cell' or 'limited'"
        )
    return mode


def _positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if result < 1:
        raise ValueError(f"{name} must be positive")
    return result


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Zhang paired")
    try:
        path = run_zhang_comparison(
            arguments.scenario,
            arguments.output,
            config_path=arguments.config,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"zhang_comparison_raw={path}")


if __name__ == "__main__":
    main()
