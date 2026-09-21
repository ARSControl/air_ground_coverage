"""Run the standalone obstacle-free Zhang et al. coverage baseline."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys
from time import perf_counter
from typing import Any, Callable

import numpy as np
import yaml

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.progress import TerminalProgress  # noqa: E402
from evaluation.zhang import (  # noqa: E402
    ZhangParameters,
    ZhangSimulation,
    paper_scalar_fields,
    uniform_grid,
)
from evaluation.zhang_io import (  # noqa: E402
    ZHANG_BASELINE_RAW_ARCHIVE_KIND,
    save_archive,
)


DEFAULT_CONFIG = Path("configs/zhang_baseline.yaml")
DEFAULT_OUTPUT = Path("output/zhang_baseline/raw.npz")
TIMING_FIELDS = ("algorithm_2",)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", "-o", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_zhang_baseline(
    config_path: str | Path,
    output_path: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run every declared composition/range condition and save raw outputs."""
    document = _load_document(config_path)
    simulation_config = _mapping(document, "simulation")
    domain_config = _mapping(document, "domain")
    field_config = _mapping(document, "field")
    sweep_config = _mapping(document, "sweep")
    algorithm_config = _mapping(document, "algorithm")

    dt = _positive_real(simulation_config.get("dt"), "simulation.dt")
    episode_count = _positive_integer(
        simulation_config.get("num_episodes") if episodes is None else episodes,
        "episodes",
    )
    step_count = _positive_integer(
        simulation_config.get("num_steps") if num_steps is None else num_steps,
        "num_steps",
    )
    random_seed = _integer(
        simulation_config.get("random_seed"), "simulation.random_seed"
    )
    bounds = _bounds(domain_config.get("bounds"))
    grid_shape = _grid_shape(domain_config.get("grid_shape"))
    points, integration_weights, resolved_shape = uniform_grid(bounds, grid_shape)
    low_resolution = _positive_real(
        field_config.get("low_resolution"), "field.low_resolution"
    )
    high_resolution = _positive_real(
        field_config.get("high_resolution"), "field.high_resolution"
    )
    low_field, high_field = paper_scalar_fields(
        points,
        low_resolution=low_resolution,
        high_resolution=high_resolution,
    )
    compositions = _compositions(sweep_config.get("compositions"))
    sensing_ranges = _positive_real_sequence(
        sweep_config.get("ground_sensing_ranges"),
        "sweep.ground_sensing_ranges",
    )
    conditions = tuple(
        (aerial_count, ground_count, sensing_range)
        for aerial_count, ground_count in compositions
        for sensing_range in sensing_ranges
    )
    condition_names = tuple(
        f"A{aerial}/G{ground}-R{sensing:g}" for aerial, ground, sensing in conditions
    )
    maximum_aerial = max(condition[0] for condition in conditions)
    maximum_ground = max(condition[1] for condition in conditions)
    parameters_base = {
        "dt": dt,
        "deployment_gain": _positive_real(
            algorithm_config.get("deployment_gain"), "algorithm.deployment_gain"
        ),
        "deployment_tolerance": _positive_real(
            algorithm_config.get("deployment_tolerance"),
            "algorithm.deployment_tolerance",
        ),
        "deployment_max_steps": _positive_integer(
            algorithm_config.get("deployment_max_steps"),
            "algorithm.deployment_max_steps",
        ),
        "consensus_gain": _positive_real(
            algorithm_config.get("consensus_gain"), "algorithm.consensus_gain"
        ),
        "consensus_iterations": _positive_integer(
            algorithm_config.get("consensus_iterations"),
            "algorithm.consensus_iterations",
        ),
        "weight_gain": _positive_real(
            algorithm_config.get("weight_gain"), "algorithm.weight_gain"
        ),
        "aerial_gain": _positive_real(
            algorithm_config.get("aerial_gain"), "algorithm.aerial_gain"
        ),
        "transfer_gain": _positive_real(
            algorithm_config.get("transfer_gain"), "algorithm.transfer_gain"
        ),
        "ground_gain": _positive_real(
            algorithm_config.get("ground_gain"), "algorithm.ground_gain"
        ),
    }

    total_steps = len(conditions) * episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "initializing Zhang baseline sweep")

    deployment_initial_runs: list[list[np.ndarray]] = []
    aerial_runs: list[list[np.ndarray]] = []
    ground_runs: list[list[np.ndarray]] = []
    weight_runs: list[list[np.ndarray]] = []
    mass_runs: list[list[np.ndarray]] = []
    load_runs: list[list[np.ndarray]] = []
    parent_runs: list[list[np.ndarray]] = []
    cost_runs: list[list[np.ndarray]] = []
    travel_runs: list[list[np.ndarray]] = []
    transfer_runs: list[list[np.ndarray]] = []
    timing_runs: list[list[np.ndarray]] = []
    target_load_runs: list[list[np.ndarray]] = []
    target_mass_runs: list[list[np.ndarray]] = []
    deployment_step_runs: list[list[int]] = []
    consensus_load_runs: list[list[np.ndarray]] = []
    consensus_ratio_runs: list[list[np.ndarray]] = []

    for condition_index, (aerial_count, ground_count, sensing_range) in enumerate(
        conditions
    ):
        composition_index = condition_index // len(sensing_ranges)
        condition_deployment_initial: list[np.ndarray] = []
        condition_aerial: list[np.ndarray] = []
        condition_ground: list[np.ndarray] = []
        condition_weights: list[np.ndarray] = []
        condition_masses: list[np.ndarray] = []
        condition_loads: list[np.ndarray] = []
        condition_parents: list[np.ndarray] = []
        condition_costs: list[np.ndarray] = []
        condition_travel: list[np.ndarray] = []
        condition_transfers: list[np.ndarray] = []
        condition_timings: list[np.ndarray] = []
        condition_target_loads: list[np.ndarray] = []
        condition_target_masses: list[np.ndarray] = []
        condition_deployment_steps: list[int] = []
        condition_consensus_loads: list[np.ndarray] = []
        condition_consensus_ratios: list[np.ndarray] = []

        for episode_index in range(episode_count):
            seed = random_seed + episode_index
            initial_aerial, initial_ground = _initial_positions(
                bounds,
                seed,
                maximum_aerial=maximum_aerial,
                maximum_ground=maximum_ground,
            )
            parameters = ZhangParameters(
                **parameters_base,
                ground_sensing_range=sensing_range,
            )
            simulation = ZhangSimulation(
                points=points,
                integration_weights=integration_weights,
                grid_shape=resolved_shape,
                low_field=low_field,
                high_field=high_field,
                aerial_positions=initial_aerial[:aerial_count],
                ground_positions=initial_ground[:ground_count],
                parameters=parameters,
                # Algorithm 1 and tie-breaking streams are paired across the
                # sensing-range conditions of the same composition.
                rng=np.random.default_rng(seed + 60000 + composition_index),
            )

            aerial_history = [
                _pad_positions(simulation.aerial_positions, maximum_aerial)
            ]
            ground_history = [
                _pad_positions(simulation.ground_positions, maximum_ground)
            ]
            cell_weight_history = [_pad_vector(simulation.cell_weights, maximum_aerial)]
            cell_mass_history = [_pad_vector(simulation.cell_masses, maximum_aerial)]
            cell_load_history = [
                _pad_vector(simulation.cell_loads, maximum_aerial, fill=-1, dtype=int)
            ]
            parent_history = [
                _pad_vector(
                    simulation.ground_parents, maximum_ground, fill=-1, dtype=int
                )
            ]
            costs = [simulation.locational_cost()]
            travel = [0.0]
            transfer_counts: list[int] = []
            timings: list[tuple[float]] = []

            for step in range(step_count):
                previous_ground = simulation.ground_positions.copy()
                started = perf_counter()
                result = simulation.step()
                duration = perf_counter() - started
                aerial_history.append(
                    _pad_positions(simulation.aerial_positions, maximum_aerial)
                )
                ground_history.append(
                    _pad_positions(simulation.ground_positions, maximum_ground)
                )
                cell_weight_history.append(
                    _pad_vector(simulation.cell_weights, maximum_aerial)
                )
                cell_mass_history.append(
                    _pad_vector(result.cell_masses, maximum_aerial)
                )
                cell_load_history.append(
                    _pad_vector(
                        result.cell_loads,
                        maximum_aerial,
                        fill=-1,
                        dtype=int,
                    )
                )
                parent_history.append(
                    _pad_vector(
                        result.ground_parents,
                        maximum_ground,
                        fill=-1,
                        dtype=int,
                    )
                )
                costs.append(simulation.locational_cost())
                travel.append(
                    travel[-1]
                    + float(
                        np.sum(
                            np.linalg.norm(
                                simulation.ground_positions - previous_ground, axis=1
                            )
                        )
                    )
                )
                transfer_counts.append(result.transferred_ground.size)
                timings.append((duration,))
                completed_steps += 1
                if progress_callback is not None:
                    progress_callback(
                        completed_steps,
                        total_steps,
                        f"condition={condition_names[condition_index]} "
                        f"episode={episode_index + 1}/{episode_count} "
                        f"step={step + 1}/{step_count}",
                    )

            condition_deployment_initial.append(
                _pad_positions(simulation.deployment_initial_positions, maximum_aerial)
            )
            condition_aerial.append(np.asarray(aerial_history, dtype=float))
            condition_ground.append(np.asarray(ground_history, dtype=float))
            condition_weights.append(np.asarray(cell_weight_history, dtype=float))
            condition_masses.append(np.asarray(cell_mass_history, dtype=float))
            condition_loads.append(np.asarray(cell_load_history, dtype=np.int64))
            condition_parents.append(np.asarray(parent_history, dtype=np.int64))
            condition_costs.append(np.asarray(costs, dtype=float))
            condition_travel.append(np.asarray(travel, dtype=float))
            condition_transfers.append(np.asarray(transfer_counts, dtype=np.int64))
            condition_timings.append(np.asarray(timings, dtype=float))
            condition_target_loads.append(
                _pad_vector(
                    simulation.target_loads,
                    maximum_aerial,
                    fill=-1,
                    dtype=int,
                )
            )
            condition_target_masses.append(
                _pad_vector(simulation.target_masses, maximum_aerial)
            )
            condition_deployment_steps.append(simulation.deployment_steps)
            condition_consensus_loads.append(
                _pad_matrix(
                    simulation.allocation.load_history,
                    maximum_aerial,
                    fill=-1,
                    dtype=int,
                )
            )
            condition_consensus_ratios.append(
                _pad_matrix(simulation.allocation.ratio_history, maximum_aerial)
            )

        deployment_initial_runs.append(condition_deployment_initial)
        aerial_runs.append(condition_aerial)
        ground_runs.append(condition_ground)
        weight_runs.append(condition_weights)
        mass_runs.append(condition_masses)
        load_runs.append(condition_loads)
        parent_runs.append(condition_parents)
        cost_runs.append(condition_costs)
        travel_runs.append(condition_travel)
        transfer_runs.append(condition_transfers)
        timing_runs.append(condition_timings)
        target_load_runs.append(condition_target_loads)
        target_mass_runs.append(condition_target_masses)
        deployment_step_runs.append(condition_deployment_steps)
        consensus_load_runs.append(condition_consensus_loads)
        consensus_ratio_runs.append(condition_consensus_ratios)

    metadata = {
        "method": "zhang",
        "paper": (
            "Distributed Coverage Control for Air-Ground Robot Systems With "
            "Heterogeneous Sensing Capabilities"
        ),
        "config_path": str(Path(config_path)),
        "obstacle_free": True,
        "episodes": episode_count,
        "num_steps": step_count,
        "dt": dt,
        "condition_names": list(condition_names),
        "timing_fields": list(TIMING_FIELDS),
        "algorithm_assumptions": {
            "convex_obstacle_free_domain": True,
            "kinematics": "first-order point robots",
            "communication": "logical power-cell neighbors in one Python process",
            "aerial_field": "quantized low-resolution scalar field",
            "ground_field": "quantized high-resolution field inside sensing disk",
        },
        "resolved_config": deepcopy(document),
    }
    arrays = {
        "method_name": np.asarray("zhang"),
        "condition_names": np.asarray(condition_names, dtype="U32"),
        "aerial_counts": np.asarray([value[0] for value in conditions], dtype=np.int64),
        "ground_counts": np.asarray([value[1] for value in conditions], dtype=np.int64),
        "ground_sensing_ranges": np.asarray(
            [value[2] for value in conditions], dtype=float
        ),
        "seeds": np.arange(random_seed, random_seed + episode_count, dtype=np.int64),
        "state_times": np.arange(step_count + 1, dtype=float) * dt,
        "query_points": points,
        "integration_weights": integration_weights,
        "grid_shape": np.asarray(resolved_shape, dtype=np.int64),
        "low_field": low_field,
        "high_field": high_field,
        "obstacle_mask": np.zeros(points.shape[0], dtype=bool),
        "deployment_initial_aerial_positions": np.asarray(
            deployment_initial_runs, dtype=float
        ),
        "aerial_positions": np.asarray(aerial_runs, dtype=float),
        "ground_positions": np.asarray(ground_runs, dtype=float),
        "cell_weights": np.asarray(weight_runs, dtype=float),
        "cell_masses": np.asarray(mass_runs, dtype=float),
        "cell_loads": np.asarray(load_runs, dtype=np.int64),
        "ground_parents": np.asarray(parent_runs, dtype=np.int64),
        "target_loads": np.asarray(target_load_runs, dtype=np.int64),
        "target_masses": np.asarray(target_mass_runs, dtype=float),
        "deployment_steps": np.asarray(deployment_step_runs, dtype=np.int64),
        "consensus_load_history": np.asarray(consensus_load_runs, dtype=np.int64),
        "consensus_ratio_history": np.asarray(consensus_ratio_runs, dtype=float),
        "locational_cost": np.asarray(cost_runs, dtype=float),
        "ground_travel_distance": np.asarray(travel_runs, dtype=float),
        "ground_transfer_count": np.asarray(transfer_runs, dtype=np.int64),
        "step_timings": np.asarray(timing_runs, dtype=float),
    }
    return save_archive(
        output_path,
        kind=ZHANG_BASELINE_RAW_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _load_document(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("Zhang configuration must be a mapping")
    return value


def _mapping(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return value


def _compositions(value: Any) -> tuple[tuple[int, int], ...]:
    if not isinstance(value, list) or not value:
        raise ValueError("sweep.compositions must be a nonempty list")
    result: list[tuple[int, int]] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"sweep.compositions[{index}] must be a mapping")
        aerial = _positive_integer(item.get("aerial"), f"composition {index} aerial")
        ground = _positive_integer(item.get("ground"), f"composition {index} ground")
        if aerial <= 1 or ground <= aerial:
            raise ValueError(
                "every Zhang composition must satisfy aerial > 1 and ground > aerial"
            )
        result.append((aerial, ground))
    if len(set(result)) != len(result):
        raise ValueError("sweep.compositions must not contain duplicates")
    return tuple(result)


def _initial_positions(
    bounds: tuple[float, float, float, float],
    seed: int,
    *,
    maximum_aerial: int,
    maximum_ground: int,
) -> tuple[np.ndarray, np.ndarray]:
    x_min, x_max, y_min, y_max = bounds
    span = np.asarray((x_max - x_min, y_max - y_min), dtype=float)
    origin = np.asarray((x_min, y_min), dtype=float)
    aerial_rng = np.random.default_rng(seed + 40000)
    ground_rng = np.random.default_rng(seed + 50000)
    aerial = origin + span * (0.05 + 0.90 * aerial_rng.random((maximum_aerial, 2)))
    ground = origin + span * (0.05 + 0.90 * ground_rng.random((maximum_ground, 2)))
    return aerial, ground


def _pad_positions(values: np.ndarray, width: int) -> np.ndarray:
    result = np.full((width, 2), np.nan, dtype=float)
    result[: values.shape[0]] = values
    return result


def _pad_vector(
    values: np.ndarray,
    width: int,
    *,
    fill: float | int = np.nan,
    dtype: type = float,
) -> np.ndarray:
    result = np.full(width, fill, dtype=dtype)
    result[: values.shape[0]] = values
    return result


def _pad_matrix(
    values: np.ndarray,
    width: int,
    *,
    fill: float | int = np.nan,
    dtype: type = float,
) -> np.ndarray:
    result = np.full((values.shape[0], width), fill, dtype=dtype)
    result[:, : values.shape[1]] = values
    return result


def _bounds(value: Any) -> tuple[float, float, float, float]:
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("domain.bounds must contain four numbers")
    result = tuple(float(item) for item in value)
    if not all(np.isfinite(result)) or not (
        result[1] > result[0] and result[3] > result[2]
    ):
        raise ValueError("domain.bounds must be finite and increasing")
    return result  # type: ignore[return-value]


def _grid_shape(value: Any) -> tuple[int, int]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("domain.grid_shape must contain rows and columns")
    result = tuple(_positive_integer(item, "domain.grid_shape") for item in value)
    if min(result) < 2:
        raise ValueError("domain.grid_shape dimensions must be at least two")
    return result  # type: ignore[return-value]


def _positive_real_sequence(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{name} must be a nonempty list")
    result = tuple(_positive_real(item, name) for item in value)
    if len(set(result)) != len(result):
        raise ValueError(f"{name} must not contain duplicates")
    return result


def _positive_real(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real number")
    result = float(value)
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _positive_integer(value: Any, name: str) -> int:
    result = _integer(value, name)
    if result < 1:
        raise ValueError(f"{name} must be positive")
    return result


def _integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    return int(value)


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Zhang")
    try:
        path = run_zhang_baseline(
            arguments.config,
            arguments.output,
            episodes=arguments.episodes,
            num_steps=arguments.num_steps,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"zhang_baseline_raw={path}")


if __name__ == "__main__":
    main()
