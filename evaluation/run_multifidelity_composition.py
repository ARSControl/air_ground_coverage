"""Run paired closed-loop simulations across fixed-size team compositions."""

from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any, Callable, Iterable

import numpy as np
from scipy.interpolate import RegularGridInterpolator

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    RAW_ARCHIVE_KIND,
    save_archive,
)
from evaluation.multifidelity_metrics import normalized_density  # noqa: E402
from evaluation.progress import TerminalProgress  # noqa: E402
from src.core.base import HEDACParams  # noqa: E402
from src.coupled_config import load_coupled_configuration  # noqa: E402
from src.coupled_simulation import build_coupled_simulation  # noqa: E402
from src.simulation import CoordinatorEventType, EstimatorMode  # noqa: E402


DEFAULT_AERIAL_COUNTS = (10, 2, 0)
TIMING_FIELDS = (
    "low_collection",
    "aerial_control",
    "high_collection",
    "estimator_update",
    "ground_control",
    "total",
)
STATE_DIMENSION = 6


@dataclass(frozen=True)
class ScenarioDefinition:
    """Resolved mission and ground-geometry settings for one archive."""

    name: str
    num_steps: int
    num_obstacles: int
    obstacles_radius: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", "-c", default="configs/multifidelity_composition.yaml"
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_composition_raw.npz"),
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument(
        "--scenario",
        default=None,
        help="named composition_sweep scenario; defaults to default_scenario",
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def allocate_retention_caps(
    total_retained_samples: int,
    aerial_count: int,
    ground_count: int,
) -> tuple[int, int]:
    """Allocate one fixed active retention budget in proportion to team size.

    ``RetentionConfig`` requires positive caps, so an inactive fidelity receives
    a placeholder cap of one. It cannot consume that cap because its team is
    empty; the sum of caps belonging to active fidelities remains exactly the
    configured total.
    """
    total = _positive_integer(total_retained_samples, "total_retained_samples")
    aerial = _nonnegative_integer(aerial_count, "aerial_count")
    ground = _nonnegative_integer(ground_count, "ground_count")
    team_size = aerial + ground
    if team_size <= 0:
        raise ValueError(
            "aerial_count and ground_count must sum to a positive team size"
        )
    if total < team_size:
        raise ValueError(
            "total_retained_samples must allow at least one sample per robot"
        )
    if aerial == 0:
        return 1, total
    if ground == 0:
        return total, 1
    low = total * aerial // team_size
    high = total - low
    if low < 1 or high < 1:
        raise ValueError("active fidelities must receive positive retention caps")
    return low, high


def run_composition_sweep(
    config_path: str | Path,
    output_path: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    aerial_counts: Iterable[int] | None = None,
    scenario: str | None = None,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run paired composition episodes and save raw states and posteriors."""
    base = load_coupled_configuration(config_path)
    if base.aerial.get("estimator_mode") != EstimatorMode.MULTIFIDELITY.value:
        raise ValueError("composition sweep requires estimator_mode: multifidelity")
    protocol = _protocol(base.aerial, aerial_counts)
    episode_count = _positive_integer(
        base.aerial.num_episodes if episodes is None else episodes, "episodes"
    )
    selected_scenario = _scenario_definition(
        base.aerial,
        ground_params=base.ground,
        requested=scenario,
        num_steps_override=num_steps,
    )
    step_count = selected_scenario.num_steps
    _validate_fair_sensing_protocol(base.aerial, base.ground)

    total_robots, requested_aerial, total_retained = protocol
    ground_counts = tuple(total_robots - count for count in requested_aerial)
    names = tuple(
        f"A{aerial}/G{ground}"
        for aerial, ground in zip(requested_aerial, ground_counts, strict=True)
    )
    seeds = np.arange(
        int(base.aerial.random_seed),
        int(base.aerial.random_seed) + episode_count,
        dtype=np.int64,
    )
    composition_count = len(requested_aerial)
    total_steps = composition_count * episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "initializing paired compositions")

    aerial_runs: list[list[np.ndarray]] = []
    ground_runs: list[list[np.ndarray]] = []
    timing_runs: list[list[np.ndarray]] = []
    low_submitted_runs: list[list[np.ndarray]] = []
    high_submitted_runs: list[list[np.ndarray]] = []
    truth_field_runs: list[list[np.ndarray]] = []
    truth_density_runs: list[list[np.ndarray]] = []
    free_mask_runs: list[list[np.ndarray]] = []
    map_runs: list[list[np.ndarray]] = []
    posterior_composition: list[int] = []
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
    posterior_hyperparameter_fit_performed: list[bool] = []
    posterior_hyperparameter_fit_duration: list[float] = []
    posterior_low_kernel_length_scale: list[float] = []
    posterior_low_kernel_variance: list[float] = []
    posterior_discrepancy_kernel_length_scale: list[float] = []
    posterior_discrepancy_kernel_variance: list[float] = []
    common_query: np.ndarray | None = None
    common_weights: np.ndarray | None = None
    common_shape: tuple[int, ...] | None = None
    low_caps: list[int] = []
    high_caps: list[int] = []

    for composition_index, (aerial_count, ground_count) in enumerate(
        zip(requested_aerial, ground_counts, strict=True)
    ):
        low_cap, high_cap = allocate_retention_caps(
            total_retained, aerial_count, ground_count
        )
        low_caps.append(low_cap)
        high_caps.append(high_cap)
        aerial_episode_states: list[np.ndarray] = []
        ground_episode_states: list[np.ndarray] = []
        episode_timings: list[np.ndarray] = []
        episode_low_submitted: list[np.ndarray] = []
        episode_high_submitted: list[np.ndarray] = []
        episode_truth_fields: list[np.ndarray] = []
        episode_truth_densities: list[np.ndarray] = []
        episode_free_masks: list[np.ndarray] = []
        episode_maps: list[np.ndarray] = []

        for episode_index, seed in enumerate(seeds):
            aerial_params, ground_params = _composition_params(
                base,
                scenario=selected_scenario,
                aerial_count=aerial_count,
                ground_count=ground_count,
                low_cap=low_cap,
                high_cap=high_cap,
            )
            simulation = build_coupled_simulation(
                aerial_params, ground_params, seed=int(seed)
            )
            if simulation.coordinator is None:
                raise RuntimeError("composition sweep requires a central estimator")
            if len(simulation.aerial_team) != aerial_count:
                raise RuntimeError("constructed aerial team has the wrong size")
            if len(simulation.ground_team) != ground_count:
                raise RuntimeError("constructed ground team has the wrong size")

            query = np.asarray(simulation.ground_controller.query_points, dtype=float)
            weights = np.asarray(
                simulation.ground_controller.integration_weights, dtype=float
            )
            if common_query is None:
                common_query = np.array(query, copy=True)
                common_weights = np.array(weights, copy=True)
            elif not (
                np.array_equal(query, common_query)
                and np.array_equal(weights, common_weights)
            ):
                raise ValueError("all compositions must use one query grid")

            high_grid = np.asarray(simulation.coordinator.fields.high, dtype=float)
            map_grid = np.asarray(simulation.ground_map, dtype=np.int8)
            truth_field = _resample_grid(high_grid, query)
            free_mask = _resample_grid(map_grid == 0, query, nearest=True).astype(bool)
            truth_density = normalized_density(
                np.maximum(truth_field, 0.0) * free_mask, weights
            )
            _validate_paired_truth(
                truth_field_runs,
                composition_index,
                episode_index,
                truth_field,
            )
            episode_truth_fields.append(truth_field)
            episode_truth_densities.append(truth_density)
            episode_free_masks.append(free_mask)
            episode_maps.append(map_grid)

            aerial_history = [
                _padded_states(simulation.aerial_team.get_states(), total_robots)
            ]
            ground_history = [
                _padded_states(simulation.ground_team.get_states(), total_robots)
            ]
            step_timings: list[list[float]] = []
            step_low_submitted: list[int] = []
            step_high_submitted: list[int] = []
            last_version = 0
            for step in range(step_count):
                step_result = simulation.step(step)
                aerial_history.append(
                    _padded_states(simulation.aerial_team.get_states(), total_robots)
                )
                ground_history.append(
                    _padded_states(simulation.ground_team.get_states(), total_robots)
                )
                step_timings.append(
                    [getattr(step_result.timing, name) for name in TIMING_FIELDS]
                )
                low_count = 0
                high_count = 0
                for report in step_result.coordinator_reports:
                    if report.event_type is CoordinatorEventType.LOW_COLLECTION:
                        low_count += report.submitted_count
                    elif report.event_type is CoordinatorEventType.HIGH_COLLECTION:
                        high_count += report.submitted_count
                step_low_submitted.append(low_count)
                step_high_submitted.append(high_count)

                completed_steps += 1
                if progress_callback is not None:
                    progress_callback(
                        completed_steps,
                        total_steps,
                        f"scenario={selected_scenario.name} "
                        f"composition={names[composition_index]} "
                        f"episode={episode_index + 1}/{episode_count} "
                        f"step={step + 1}/{step_count}",
                    )
                snapshot = simulation.latest_posterior
                if snapshot is None or snapshot.version == last_version:
                    continue
                last_version = snapshot.version
                if common_shape is None:
                    common_shape = snapshot.query_shape
                elif snapshot.query_shape != common_shape:
                    raise ValueError("all compositions must use one query shape")
                posterior_composition.append(composition_index)
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
                posterior_hyperparameter_fit_performed.append(
                    snapshot.hyperparameter_fit_performed
                )
                posterior_hyperparameter_fit_duration.append(
                    snapshot.hyperparameter_fit_duration
                )
                posterior_low_kernel_length_scale.append(
                    snapshot.low_kernel_length_scale
                )
                posterior_low_kernel_variance.append(snapshot.low_kernel_variance)
                posterior_discrepancy_kernel_length_scale.append(
                    snapshot.discrepancy_kernel_length_scale
                )
                posterior_discrepancy_kernel_variance.append(
                    snapshot.discrepancy_kernel_variance
                )

            if last_version == 0:
                raise RuntimeError(
                    f"{names[composition_index]} episode {episode_index} "
                    "published no posterior"
                )
            aerial_episode_states.append(np.asarray(aerial_history, dtype=float))
            ground_episode_states.append(np.asarray(ground_history, dtype=float))
            episode_timings.append(np.asarray(step_timings, dtype=float))
            episode_low_submitted.append(np.asarray(step_low_submitted, dtype=np.int64))
            episode_high_submitted.append(
                np.asarray(step_high_submitted, dtype=np.int64)
            )

        aerial_runs.append(aerial_episode_states)
        ground_runs.append(ground_episode_states)
        timing_runs.append(episode_timings)
        low_submitted_runs.append(episode_low_submitted)
        high_submitted_runs.append(episode_high_submitted)
        truth_field_runs.append(episode_truth_fields)
        truth_density_runs.append(episode_truth_densities)
        free_mask_runs.append(episode_free_masks)
        map_runs.append(episode_maps)

    if common_query is None or common_weights is None or common_shape is None:
        raise RuntimeError("composition sweep produced no posterior snapshots")
    point_count = common_query.shape[0]
    metadata = {
        "config_path": str(Path(config_path)),
        "scenario": selected_scenario.name,
        "scenario_parameters": {
            "num_steps": selected_scenario.num_steps,
            "num_obstacles": selected_scenario.num_obstacles,
            "obstacles_radius": selected_scenario.obstacles_radius,
        },
        "compositions": list(names),
        "total_robots": total_robots,
        "total_retained_samples": total_retained,
        "episodes": episode_count,
        "num_steps": step_count,
        "dt": float(base.aerial.dt),
        "timing_fields": list(TIMING_FIELDS),
        "query_shape": list(common_shape),
        "initialization": deepcopy(
            base.aerial.get("initialization", {"position_policy": "uniform"})
        ),
        "hyperparameter_optimization": deepcopy(
            base.aerial.get("multifidelity.hyperparameter_optimization")
        ),
        "aerial_ergodic_control": {
            "mode": str(base.aerial.get("ergodic_control.mode", "centralized"))
            .strip()
            .lower(),
            "neighbor_weighting": str(
                base.aerial.get("ergodic_control.neighbor_weighting", "hard")
            )
            .strip()
            .lower(),
            "communication_range": float(base.aerial.sens_range),
        },
        "fairness_controls": {
            "equal_sensor_periods": True,
            "equal_observations_per_robot_event": True,
            "uncertainty_admission_filter_enabled": False,
            "active_retention_budget_fixed": True,
            "shared_hyperparameter_policy": True,
            "shared_initialization_policy": True,
        },
        "resolved_aerial_config": _scenario_config(
            base.aerial, selected_scenario, ground=False
        ),
        "resolved_ground_config": _scenario_config(
            base.ground, selected_scenario, ground=True
        ),
    }
    arrays = {
        "composition_names": np.asarray(names, dtype="U16"),
        "aerial_counts": np.asarray(requested_aerial, dtype=np.int64),
        "ground_counts": np.asarray(ground_counts, dtype=np.int64),
        "retention_low_caps": np.asarray(low_caps, dtype=np.int64),
        "retention_high_caps": np.asarray(high_caps, dtype=np.int64),
        "seeds": seeds,
        "state_times": np.arange(step_count + 1, dtype=float) * float(base.aerial.dt),
        "query_points": common_query,
        "integration_weights": common_weights,
        "high_truth": np.asarray(truth_field_runs, dtype=float),
        "truth_density": np.asarray(truth_density_runs, dtype=float),
        "free_mask": np.asarray(free_mask_runs, dtype=bool),
        "map_grid": np.asarray(map_runs, dtype=np.int8),
        "aerial_states": np.asarray(aerial_runs, dtype=float),
        "ground_states": np.asarray(ground_runs, dtype=float),
        "step_timings": np.asarray(timing_runs, dtype=float),
        "low_submitted": np.asarray(low_submitted_runs, dtype=np.int64),
        "high_submitted": np.asarray(high_submitted_runs, dtype=np.int64),
        "posterior_composition": np.asarray(posterior_composition, dtype=np.int64),
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
        "posterior_hyperparameter_fit_performed": np.asarray(
            posterior_hyperparameter_fit_performed, dtype=bool
        ),
        "posterior_hyperparameter_fit_duration": np.asarray(
            posterior_hyperparameter_fit_duration, dtype=float
        ),
        "posterior_low_kernel_length_scale": np.asarray(
            posterior_low_kernel_length_scale, dtype=float
        ),
        "posterior_low_kernel_variance": np.asarray(
            posterior_low_kernel_variance, dtype=float
        ),
        "posterior_discrepancy_kernel_length_scale": np.asarray(
            posterior_discrepancy_kernel_length_scale, dtype=float
        ),
        "posterior_discrepancy_kernel_variance": np.asarray(
            posterior_discrepancy_kernel_variance, dtype=float
        ),
    }
    return save_archive(
        output_path, kind=RAW_ARCHIVE_KIND, metadata=metadata, arrays=arrays
    )


def _protocol(
    params: HEDACParams, aerial_counts: Iterable[int] | None
) -> tuple[int, tuple[int, ...], int]:
    total = _positive_integer(
        params.get("composition_sweep.total_robots"), "composition_sweep.total_robots"
    )
    raw_counts = (
        params.get("composition_sweep.aerial_counts")
        if aerial_counts is None
        else tuple(aerial_counts)
    )
    if not isinstance(raw_counts, (list, tuple)) or not raw_counts:
        raise ValueError("composition_sweep.aerial_counts must be a nonempty sequence")
    counts = tuple(
        _nonnegative_integer(value, "composition_sweep.aerial_counts")
        for value in raw_counts
    )
    if len(set(counts)) != len(counts):
        raise ValueError("composition_sweep.aerial_counts must be unique")
    if any(value > total for value in counts):
        raise ValueError("aerial robot counts must not exceed total_robots")
    retained = _positive_integer(
        params.get("composition_sweep.total_retained_samples"),
        "composition_sweep.total_retained_samples",
    )
    return total, counts, retained


def _validate_fair_sensing_protocol(aerial: HEDACParams, ground: HEDACParams) -> None:
    default_initialization = {"position_policy": "uniform"}
    if aerial.get("initialization", default_initialization) != ground.get(
        "initialization", default_initialization
    ):
        raise ValueError("composition sweep requires one shared initialization policy")
    if not np.isclose(
        float(aerial.get("multifidelity.aerial_sensor_period")),
        float(aerial.get("multifidelity.ground_sensor_period")),
    ):
        raise ValueError("composition sweep requires equal LOW/HIGH sensor periods")
    if int(aerial.get("gpr.obs_per_step")) != int(ground.get("gpr.obs_per_step")):
        raise ValueError(
            "composition sweep requires equal observations per robot sensing event"
        )
    if aerial.get("multifidelity.retention.take_threshold", None) is not None:
        raise ValueError("composition sweep requires retention.take_threshold: null")


def _scenario_definition(
    params: HEDACParams,
    *,
    ground_params: HEDACParams,
    requested: str | None,
    num_steps_override: int | None,
) -> ScenarioDefinition:
    scenarios = params.get("composition_sweep.scenarios", None)
    if scenarios is None:
        if requested is not None:
            raise ValueError(
                f"unknown scenario {requested!r}; configuration declares no scenarios"
            )
        name = "default"
        values: dict[str, Any] = {}
    else:
        if not isinstance(scenarios, dict) or not scenarios:
            raise ValueError("composition_sweep.scenarios must be a nonempty mapping")
        name = (
            params.get("composition_sweep.default_scenario")
            if requested is None
            else requested
        )
        if not isinstance(name, str) or not name.strip():
            raise ValueError("composition_sweep.default_scenario must be nonempty")
        name = name.strip()
        if name not in scenarios:
            choices = ", ".join(str(value) for value in scenarios)
            raise ValueError(
                f"unknown scenario {name!r}; available scenarios: {choices}"
            )
        values = scenarios[name]
        if not isinstance(values, dict):
            raise TypeError(f"scenario {name!r} must be a mapping")
        required = {"num_steps", "num_obstacles", "obstacles_radius"}
        missing = required.difference(values)
        if missing:
            raise ValueError(
                f"scenario {name!r} is missing: {', '.join(sorted(missing))}"
            )
        unexpected = set(values).difference(required)
        if unexpected:
            raise ValueError(
                f"scenario {name!r} has unsupported keys: "
                f"{', '.join(sorted(unexpected))}"
            )

    steps = _positive_integer(
        (
            values.get("num_steps", params.num_steps)
            if num_steps_override is None
            else num_steps_override
        ),
        "scenario.num_steps",
    )
    obstacles = _nonnegative_integer(
        values.get("num_obstacles", ground_params.num_obstacles),
        "scenario.num_obstacles",
    )
    radius = _positive_real(
        values.get("obstacles_radius", ground_params.obstacles_radius),
        "scenario.obstacles_radius",
    )
    return ScenarioDefinition(
        name=name,
        num_steps=steps,
        num_obstacles=obstacles,
        obstacles_radius=radius,
    )


def _composition_params(
    base,
    *,
    scenario: ScenarioDefinition,
    aerial_count: int,
    ground_count: int,
    low_cap: int,
    high_cap: int,
) -> tuple[HEDACParams, HEDACParams]:
    aerial = _scenario_config(base.aerial, scenario, ground=False)
    ground = _scenario_config(base.ground, scenario, ground=True)
    for document in (aerial, ground):
        _set_nested(document, "multifidelity.retention.max_low_samples", low_cap)
        _set_nested(document, "multifidelity.retention.max_high_samples", high_cap)
    _set_nested(aerial, "simulation.num_agents", aerial_count)
    _set_nested(ground, "simulation.num_agents", ground_count)
    return HEDACParams.from_dict(aerial), HEDACParams.from_dict(ground)


def _scenario_config(
    params: HEDACParams, scenario: ScenarioDefinition, *, ground: bool
) -> dict[str, Any]:
    document = deepcopy(params.to_dict())
    _set_nested(document, "simulation.num_steps", scenario.num_steps)
    if ground:
        _set_nested(document, "simulation.num_obstacles", scenario.num_obstacles)
        _set_nested(document, "simulation.obstacles_radius", scenario.obstacles_radius)
    return document


def _set_nested(document: dict[str, Any], dotted_key: str, value: Any) -> None:
    components = dotted_key.split(".")
    current = document
    for component in components[:-1]:
        current = current.setdefault(component, {})
    current[components[-1]] = value


def _padded_states(states: np.ndarray, width: int) -> np.ndarray:
    values = np.asarray(states, dtype=float)
    if values.size == 0:
        values = np.empty((0, STATE_DIMENSION), dtype=float)
    if values.ndim != 2 or values.shape[1] != STATE_DIMENSION:
        raise ValueError("team states must have shape (robots, 6)")
    if values.shape[0] > width:
        raise ValueError("team state count exceeds configured total team size")
    padded = np.full((width, STATE_DIMENSION), np.nan, dtype=float)
    padded[: values.shape[0]] = values
    return padded


def _validate_paired_truth(
    completed: list[list[np.ndarray]],
    composition_index: int,
    episode_index: int,
    truth: np.ndarray,
) -> None:
    if composition_index == 0:
        return
    reference = completed[0][episode_index]
    if not np.array_equal(reference, truth):
        raise RuntimeError("paired compositions produced different truth fields")


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


def _positive_integer(value: Any, name: str) -> int:
    result = _nonnegative_integer(value, name)
    if result < 1:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
        raise TypeError(f"{name} must be an integer")
    result = int(value)
    if result < 0:
        raise ValueError(f"{name} must be nonnegative")
    return result


def _positive_real(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _declared_scenario_names(config_path: str | Path) -> tuple[str, ...]:
    configuration = load_coupled_configuration(config_path)
    scenarios = configuration.aerial.get("composition_sweep.scenarios", None)
    if scenarios is None:
        return ()
    if not isinstance(scenarios, dict) or not scenarios:
        raise ValueError("composition_sweep.scenarios must be a nonempty mapping")
    return tuple(str(name) for name in scenarios)


def _scenario_output_path(output_path: Path, scenario: str) -> Path:
    if output_path.suffix != ".npz":
        raise ValueError("archive output path must end in .npz")
    stem = output_path.stem
    if stem.endswith("_raw"):
        stem = f"{stem[:-4]}_{scenario}_raw"
    else:
        stem = f"{stem}_{scenario}"
    return output_path.with_name(f"{stem}{output_path.suffix}")


def main() -> None:
    arguments = parse_args()
    if arguments.scenario is None:
        declared = _declared_scenario_names(arguments.config)
        requested_scenarios: tuple[str | None, ...] = declared or (None,)
    else:
        requested_scenarios = (arguments.scenario,)

    multiple = len(requested_scenarios) > 1
    for requested in requested_scenarios:
        output = (
            _scenario_output_path(arguments.output, str(requested))
            if multiple
            else arguments.output
        )
        label = "Composition" if requested is None else f"Composition {requested}"
        progress = None if arguments.no_progress else TerminalProgress(label)
        try:
            path = run_composition_sweep(
                arguments.config,
                output,
                episodes=arguments.episodes,
                num_steps=arguments.num_steps,
                scenario=requested,
                progress_callback=None if progress is None else progress.update,
            )
        finally:
            if progress is not None:
                progress.close()
        print(f"composition_raw={path}")


if __name__ == "__main__":
    main()
