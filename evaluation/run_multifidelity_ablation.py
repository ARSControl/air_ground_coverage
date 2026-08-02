"""Run ablation simulations and save raw states/posteriors without evaluation."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys
from typing import Any, Callable

import numpy as np
from scipy.interpolate import RegularGridInterpolator

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_ablation_io import (  # noqa: E402
    RAW_ARCHIVE_KIND,
    save_archive,
)
from evaluation.multifidelity_metrics import normalized_density  # noqa: E402
from evaluation.progress import TerminalProgress  # noqa: E402
from src.core.base import HEDACParams  # noqa: E402
from src.coupled_config import load_coupled_configuration  # noqa: E402
from src.coupled_simulation import build_coupled_simulation  # noqa: E402


VARIANTS = (
    "full",
    "no_discrepancy",
    "no_high_to_aerial",
    "no_uncertainty",
)
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
    parser.add_argument(
        "--config", "-c", default="configs/multifidelity_ablation.yaml"
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_ablation_raw.npz"),
    )
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--num-steps", type=int, default=None)
    parser.add_argument(
        "--variants", nargs="+", choices=VARIANTS, default=list(VARIANTS)
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def run_ablation_simulations(
    config_path: str | Path,
    output_path: str | Path,
    *,
    episodes: int | None = None,
    num_steps: int | None = None,
    variants: tuple[str, ...] | list[str] = VARIANTS,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Run requested variants and atomically save one self-describing raw NPZ."""
    requested = tuple(variants)
    if not requested or len(set(requested)) != len(requested):
        raise ValueError("variants must be nonempty and unique")
    if any(name not in VARIANTS for name in requested):
        raise ValueError(f"variants must be selected from {VARIANTS}")
    base = load_coupled_configuration(config_path)
    episode_count = base.aerial.num_episodes if episodes is None else episodes
    step_count = base.aerial.num_steps if num_steps is None else num_steps
    if (
        isinstance(episode_count, bool)
        or not isinstance(episode_count, (int, np.integer))
        or episode_count < 1
    ):
        raise ValueError("episodes must be a positive integer")
    if (
        isinstance(step_count, bool)
        or not isinstance(step_count, (int, np.integer))
        or step_count < 1
    ):
        raise ValueError("num_steps must be a positive integer")
    episode_count = int(episode_count)
    step_count = int(step_count)
    total_steps = len(requested) * episode_count * step_count
    completed_steps = 0
    if progress_callback is not None:
        progress_callback(0, total_steps, "initializing paired simulations")

    if any(name in {"no_discrepancy", "no_high_to_aerial"} for name in requested):
        if base.aerial.get("multifidelity.hyperparameter_optimization.enabled", False):
            raise ValueError(
                "ablation isolation currently requires hyperparameter optimization "
                "to be disabled for every variant"
            )
    if (
        "no_high_to_aerial" in requested
        and base.aerial.get("multifidelity.retention.take_threshold", None) is not None
    ):
        raise ValueError(
            "no_high_to_aerial requires retention.take_threshold: null so HIGH "
            "uncertainty cannot affect LOW observation admission"
        )

    seeds = np.arange(
        int(base.aerial.random_seed),
        int(base.aerial.random_seed) + episode_count,
        dtype=np.int64,
    )
    aerial_runs: list[list[np.ndarray]] = []
    ground_runs: list[list[np.ndarray]] = []
    timing_runs: list[list[np.ndarray]] = []
    truth_field_runs: list[list[np.ndarray]] = []
    truth_density_runs: list[list[np.ndarray]] = []
    free_mask_runs: list[list[np.ndarray]] = []
    map_runs: list[list[np.ndarray]] = []
    posterior_variant: list[int] = []
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
    common_shape: tuple[int, ...] | None = None
    ground_fov: list[float] = []
    ground_range: list[float] = []

    for variant_index, variant in enumerate(requested):
        aerial_episode_states: list[np.ndarray] = []
        ground_episode_states: list[np.ndarray] = []
        episode_timings: list[np.ndarray] = []
        episode_truth_fields: list[np.ndarray] = []
        episode_truth_densities: list[np.ndarray] = []
        episode_free_masks: list[np.ndarray] = []
        episode_maps: list[np.ndarray] = []
        _, ground_template = _variant_params(base, variant)
        ground_fov.append(float(ground_template.fov_deg))
        ground_range.append(float(ground_template.fov_depth))

        for episode_index, seed in enumerate(seeds):
            # The builder normalizes a few settings in place, so every episode
            # receives fresh parameter objects.  This also prevents state from
            # one paired run leaking into the next one.
            aerial_params, ground_params = _variant_params(base, variant)
            simulation = build_coupled_simulation(
                aerial_params, ground_params, seed=int(seed)
            )
            if simulation.coordinator is None:
                raise RuntimeError("ablation study requires multifidelity mode")
            if len(simulation.aerial_team) == 0 or len(simulation.ground_team) == 0:
                raise ValueError("ablation settings require both robot teams")
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
                raise ValueError("all ablation variants must use one query grid")

            initial_snapshot = simulation.latest_posterior
            if initial_snapshot is not None:
                raise RuntimeError("new simulations must begin without a posterior")
            high_grid = np.asarray(simulation.coordinator.fields.high, dtype=float)
            map_grid = np.asarray(simulation.hedac.map, dtype=np.int8)
            truth_field = _resample_grid(high_grid, query)
            free_mask = _resample_grid(map_grid == 0, query, nearest=True).astype(bool)
            truth_density = normalized_density(
                np.maximum(truth_field, 0.0) * free_mask, weights
            )
            episode_truth_fields.append(truth_field)
            episode_truth_densities.append(truth_density)
            episode_free_masks.append(free_mask)
            episode_maps.append(map_grid)

            aerial_history = [simulation.aerial_team.get_states()]
            ground_history = [simulation.ground_team.get_states()]
            step_timings: list[list[float]] = []
            last_version = 0
            for step in range(step_count):
                step_result = simulation.step(step)
                aerial_history.append(simulation.aerial_team.get_states())
                ground_history.append(simulation.ground_team.get_states())
                step_timings.append(
                    [getattr(step_result.timing, name) for name in TIMING_FIELDS]
                )
                completed_steps += 1
                if progress_callback is not None:
                    progress_callback(
                        completed_steps,
                        total_steps,
                        f"variant={variant} episode={episode_index + 1}/"
                        f"{episode_count} step={step + 1}/{step_count}",
                    )
                snapshot = simulation.latest_posterior
                if snapshot is None or snapshot.version == last_version:
                    continue
                last_version = snapshot.version
                if common_shape is None:
                    common_shape = snapshot.query_shape
                elif snapshot.query_shape != common_shape:
                    raise ValueError("all ablation variants must use one query shape")
                posterior_variant.append(variant_index)
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
                posterior_low_only_duration.append(
                    snapshot.low_only_projection_duration
                )
            aerial_episode_states.append(np.asarray(aerial_history, dtype=float))
            ground_episode_states.append(np.asarray(ground_history, dtype=float))
            episode_timings.append(np.asarray(step_timings, dtype=float))

        aerial_runs.append(aerial_episode_states)
        ground_runs.append(ground_episode_states)
        timing_runs.append(episode_timings)
        truth_field_runs.append(episode_truth_fields)
        truth_density_runs.append(episode_truth_densities)
        free_mask_runs.append(episode_free_masks)
        map_runs.append(episode_maps)

    if common_query is None or common_weights is None or common_shape is None:
        raise RuntimeError("ablation study produced no posterior snapshots")
    point_count = common_query.shape[0]
    metadata = {
        "config_path": str(Path(config_path)),
        "episodes": episode_count,
        "num_steps": step_count,
        "dt": float(base.aerial.dt),
        "variants": list(requested),
        "timing_fields": list(TIMING_FIELDS),
        "query_shape": list(common_shape),
        "resolved_aerial_config": base.aerial.to_dict(),
        "resolved_ground_config": base.ground.to_dict(),
        "ablation_constraints": {
            "hyperparameter_optimization_enabled": bool(
                base.aerial.get(
                    "multifidelity.hyperparameter_optimization.enabled", False
                )
            ),
            "uncertainty_admission_filter_enabled": base.aerial.get(
                "multifidelity.retention.take_threshold", None
            )
            is not None,
        },
    }
    arrays = {
        "variant_names": np.asarray(requested, dtype="U32"),
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
        "ground_fov_degrees": np.asarray(ground_fov, dtype=float),
        "ground_sensing_range": np.asarray(ground_range, dtype=float),
        "posterior_variant": np.asarray(posterior_variant, dtype=np.int64),
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
    return save_archive(
        output_path, kind=RAW_ARCHIVE_KIND, metadata=metadata, arrays=arrays
    )


def _variant_params(configuration, variant: str) -> tuple[HEDACParams, HEDACParams]:
    aerial = deepcopy(configuration.aerial.to_dict())
    ground = deepcopy(configuration.ground.to_dict())
    if variant == "no_discrepancy":
        _set_nested(aerial, "multifidelity.discrepancy_enabled", False)
        _set_nested(ground, "multifidelity.discrepancy_enabled", False)
    elif variant == "no_high_to_aerial":
        _set_nested(
            aerial,
            "multifidelity.aerial_target.source",
            "low_only_projection",
        )
        _set_nested(
            ground,
            "multifidelity.aerial_target.source",
            "low_only_projection",
        )
    elif variant == "no_uncertainty":
        _set_nested(aerial, "multifidelity.aerial_target.lambda_uncertainty", 0.0)
        _set_nested(ground, "multifidelity.aerial_target.lambda_uncertainty", 0.0)
    return HEDACParams.from_dict(aerial), HEDACParams.from_dict(ground)


def _set_nested(document: dict[str, Any], dotted_key: str, value: Any) -> None:
    components = dotted_key.split(".")
    current = document
    for component in components[:-1]:
        current = current.setdefault(component, {})
    current[components[-1]] = value


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
    progress = None if arguments.no_progress else TerminalProgress("Simulation")
    try:
        path = run_ablation_simulations(
            arguments.config,
            arguments.output,
            episodes=arguments.episodes,
            num_steps=arguments.num_steps,
            variants=arguments.variants,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"raw_results={path}")


if __name__ == "__main__":
    main()
