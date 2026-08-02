"""Evaluate a saved raw ablation archive and write metric-only results."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_ablation_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
)
from evaluation.multifidelity_metrics import (  # noqa: E402
    calibration_95,
    covered_probability_mass,
    footprint_normalized_coverage,
    kl_divergence,
    maximum_density_mass_for_area,
    nrmse,
    sector_footprint_area,
)
from evaluation.progress import TerminalProgress  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_ablation_raw.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_ablation_evaluated.npz"),
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def evaluate_ablation_archive(
    raw_path: str | Path,
    output_path: str | Path,
    *,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute metrics without importing or rerunning simulation code."""
    raw_metadata, raw = load_archive(raw_path, expected_kind=RAW_ARCHIVE_KIND)
    variants = raw["variant_names"]
    seeds = raw["seeds"]
    variant_count = variants.size
    episode_count = seeds.size
    state_times = raw["state_times"]
    posterior_variant = raw["posterior_variant"].astype(int)
    posterior_episode = raw["posterior_episode"].astype(int)
    posterior_time = raw["posterior_time"]
    record_count = posterior_variant.size
    coverage_work = variant_count * episode_count * (state_times.size + 1)
    total_work = record_count + coverage_work
    completed_work = 0
    if progress_callback is not None:
        progress_callback(0, total_work, "loading saved posterior records")
    for name in (
        "posterior_episode",
        "posterior_step",
        "posterior_time",
        "posterior_version",
    ):
        if raw[name].shape != (record_count,):
            raise ValueError(f"{name} must match posterior record count")

    kl_values = np.empty(record_count, dtype=float)
    nrmse_values = np.empty(record_count, dtype=float)
    calibration_values = np.empty(record_count, dtype=float)
    for record in range(record_count):
        variant = posterior_variant[record]
        episode = posterior_episode[record]
        truth_field = raw["high_truth"][variant, episode]
        truth_density = raw["truth_density"][variant, episode]
        free = raw["free_mask"][variant, episode].astype(bool)
        kl_values[record] = kl_divergence(
            truth_density,
            raw["posterior_density"][record],
            raw["integration_weights"],
        )
        nrmse_values[record] = nrmse(
            truth_field[free], raw["posterior_mean"][record, free]
        )
        calibration_values[record] = calibration_95(
            truth_field[free],
            raw["posterior_mean"][record, free],
            raw["posterior_variance"][record, free],
        )
        completed_work += 1
        if progress_callback is not None:
            progress_callback(
                completed_work,
                total_work,
                f"posterior metrics {record + 1}/{record_count}",
            )

    coverage_shape = (variant_count, episode_count, state_times.size)
    covered_mass = np.empty(coverage_shape, dtype=float)
    coverage = np.empty(coverage_shape, dtype=float)
    coverage_oracle_mass = np.empty((variant_count, episode_count), dtype=float)
    coverage_area_budget = np.empty((variant_count, episode_count), dtype=float)
    coverage_free_area = np.empty((variant_count, episode_count), dtype=float)
    coverage_area_fraction = np.empty((variant_count, episode_count), dtype=float)
    for variant in range(variant_count):
        for episode in range(episode_count):
            free = raw["free_mask"][variant, episode].astype(bool)
            robot_count = raw["ground_states"].shape[3]
            nominal_area = robot_count * sector_footprint_area(
                fov_degrees=float(raw["ground_fov_degrees"][variant]),
                sensing_range=float(raw["ground_sensing_range"][variant]),
            )
            free_area = float(np.sum(raw["integration_weights"][free], dtype=float))
            area_budget = min(nominal_area, free_area)
            oracle_mass = maximum_density_mass_for_area(
                raw["truth_density"][variant, episode],
                raw["integration_weights"],
                area_budget=area_budget,
                free_mask=free,
            )
            coverage_area_budget[variant, episode] = area_budget
            coverage_free_area[variant, episode] = free_area
            coverage_area_fraction[variant, episode] = area_budget / free_area
            coverage_oracle_mass[variant, episode] = oracle_mass
            completed_work += 1
            if progress_callback is not None:
                progress_callback(
                    completed_work,
                    total_work,
                    f"coverage oracle variant={variant + 1}/{variant_count} "
                    f"episode={episode + 1}/{episode_count}",
                )
            for time_index, states in enumerate(raw["ground_states"][variant, episode]):
                mass = covered_probability_mass(
                    raw["query_points"],
                    raw["truth_density"][variant, episode],
                    raw["integration_weights"],
                    states,
                    fov_degrees=float(raw["ground_fov_degrees"][variant]),
                    sensing_range=float(raw["ground_sensing_range"][variant]),
                )
                covered_mass[variant, episode, time_index] = mass
                coverage[variant, episode, time_index] = (
                    footprint_normalized_coverage(mass, oracle_mass)
                )
                completed_work += 1
                if progress_callback is not None:
                    progress_callback(
                        completed_work,
                        total_work,
                        f"coverage variant={variant + 1}/{variant_count} "
                        f"episode={episode + 1}/{episode_count} "
                        f"state={time_index + 1}/{state_times.size}",
                    )

    metric_records = {
        "kl": kl_values,
        "nrmse": nrmse_values,
        "calibration_95": calibration_values,
    }
    final_metrics = {
        name: np.full((variant_count, episode_count), np.nan, dtype=float)
        for name in metric_records
    }
    time_average_metrics = {
        name: np.full((variant_count, episode_count), np.nan, dtype=float)
        for name in metric_records
    }
    for variant in range(variant_count):
        for episode in range(episode_count):
            selected = np.flatnonzero(
                (posterior_variant == variant) & (posterior_episode == episode)
            )
            if selected.size == 0:
                continue
            order = selected[np.argsort(posterior_time[selected])]
            times = posterior_time[order]
            for name, values in metric_records.items():
                series = values[order]
                final_metrics[name][variant, episode] = series[-1]
                time_average_metrics[name][variant, episode] = _time_average(
                    times, series, end_time=float(state_times[-1])
                )

    coverage_final = coverage[:, :, -1]
    coverage_time_average = np.trapezoid(coverage, state_times, axis=2) / (
        state_times[-1] - state_times[0]
    )
    covered_mass_final = covered_mass[:, :, -1]
    covered_mass_time_average = np.trapezoid(
        covered_mass, state_times, axis=2
    ) / (state_times[-1] - state_times[0])
    step_timings = raw["step_timings"]
    timing_percentiles = np.stack(
        (
            np.median(step_timings, axis=2),
            np.percentile(step_timings, 95.0, axis=2),
            np.max(step_timings, axis=2),
        ),
        axis=-1,
    )

    metadata = {
        "raw_source": str(Path(raw_path)),
        "variants": [str(value) for value in variants.tolist()],
        "episodes": episode_count,
        "num_steps": int(raw_metadata["num_steps"]),
        "dt": float(raw_metadata["dt"]),
        "timing_fields": raw_metadata["timing_fields"],
        "timing_statistics": ["median", "p95", "maximum"],
        "metric_definitions": {
            "kl": "weighted D_KL(truth_density || posterior_density)",
            "nrmse": "free-space RMSE(high_mean, high_truth) / range(high_truth)",
            "calibration_95": "free-space fraction of truth inside mean +/- 1.96 std",
            "covered_probability_mass": (
                "truth mass inside the union of actual ground FOV sectors"
            ),
            "coverage": (
                "covered_probability_mass divided by the truth mass in the "
                "densest free-space subset with area min(N * sector_area, "
                "free_space_area)"
            ),
            "coverage_oracle_mass": (
                "truth mass in the densest free-space subset under the nominal "
                "combined footprint-area budget"
            ),
            "coverage_area_fraction": (
                "nominal combined footprint-area budget divided by free-space area"
            ),
        },
    }
    arrays = {
        "variant_names": variants,
        "seeds": seeds,
        "state_times": state_times,
        "posterior_variant": posterior_variant,
        "posterior_episode": posterior_episode,
        "posterior_step": raw["posterior_step"],
        "posterior_time": posterior_time,
        "posterior_version": raw["posterior_version"],
        "kl": kl_values,
        "nrmse": nrmse_values,
        "calibration_95": calibration_values,
        "covered_probability_mass": covered_mass,
        "coverage": coverage,
        "coverage_oracle_mass": coverage_oracle_mass,
        "coverage_area_budget": coverage_area_budget,
        "coverage_free_area": coverage_free_area,
        "coverage_area_fraction": coverage_area_fraction,
        "final_kl": final_metrics["kl"],
        "final_nrmse": final_metrics["nrmse"],
        "final_calibration_95": final_metrics["calibration_95"],
        "final_covered_probability_mass": covered_mass_final,
        "final_coverage": coverage_final,
        "time_average_kl": time_average_metrics["kl"],
        "time_average_nrmse": time_average_metrics["nrmse"],
        "time_average_calibration_95": time_average_metrics["calibration_95"],
        "time_average_covered_probability_mass": covered_mass_time_average,
        "time_average_coverage": coverage_time_average,
        "timing_percentiles": timing_percentiles,
    }
    return save_archive(
        output_path,
        kind=EVALUATED_ARCHIVE_KIND,
        metadata=metadata,
        arrays=arrays,
    )


def _time_average(
    times: np.ndarray, values: np.ndarray, *, end_time: float
) -> float:
    if end_time < times[-1]:
        raise ValueError("end_time must not precede the final metric record")
    if end_time > times[-1]:
        times = np.append(times, end_time)
        values = np.append(values, values[-1])
    if values.size == 1 or times[-1] == times[0]:
        return float(values[-1])
    return float(np.trapezoid(values, times) / (times[-1] - times[0]))


def main() -> None:
    arguments = parse_args()
    progress = None if arguments.no_progress else TerminalProgress("Evaluation")
    try:
        path = evaluate_ablation_archive(
            arguments.input,
            arguments.output,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"evaluated_results={path}")


if __name__ == "__main__":
    main()
