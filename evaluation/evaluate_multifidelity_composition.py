"""Evaluate reconstruction metrics from a raw team-composition archive."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Callable

import numpy as np

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation.multifidelity_composition_io import (  # noqa: E402
    EVALUATED_ARCHIVE_KIND,
    RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
)
from evaluation.multifidelity_metrics import (  # noqa: E402
    calibration_95,
    kl_divergence,
    negative_log_predictive_density,
    nrmse,
)
from evaluation.progress import TerminalProgress  # noqa: E402


HYPERPARAMETER_RECORD_FIELDS = (
    "posterior_hyperparameter_fit_performed",
    "posterior_hyperparameter_fit_duration",
    "posterior_low_kernel_length_scale",
    "posterior_low_kernel_variance",
    "posterior_discrepancy_kernel_length_scale",
    "posterior_discrepancy_kernel_variance",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        "-i",
        type=Path,
        default=Path("output/multifidelity_composition_raw.npz"),
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=Path("output/multifidelity_composition_evaluated.npz"),
    )
    parser.add_argument(
        "--no-progress", action="store_true", help="disable terminal progress output"
    )
    return parser.parse_args()


def evaluate_composition_archive(
    raw_path: str | Path,
    output_path: str | Path,
    *,
    progress_callback: Callable[[int, int, str], None] | None = None,
) -> Path:
    """Compute posterior reconstruction metrics without rerunning simulation."""
    raw_metadata, raw = load_archive(raw_path, expected_kind=RAW_ARCHIVE_KIND)
    names = raw["composition_names"]
    seeds = raw["seeds"]
    composition_count = names.size
    episode_count = seeds.size
    state_times = raw["state_times"]
    record_composition = raw["posterior_composition"].astype(int)
    record_episode = raw["posterior_episode"].astype(int)
    record_time = raw["posterior_time"]
    record_count = record_composition.size
    _validate_raw_shapes(raw, composition_count, episode_count, record_count)
    if progress_callback is not None:
        progress_callback(0, record_count, "loading saved posterior records")

    kl_values = np.empty(record_count, dtype=float)
    nrmse_values = np.empty(record_count, dtype=float)
    nlpd_values = np.empty(record_count, dtype=float)
    calibration_values = np.empty(record_count, dtype=float)
    for record in range(record_count):
        composition = record_composition[record]
        episode = record_episode[record]
        free = raw["free_mask"][composition, episode].astype(bool)
        kl_values[record] = kl_divergence(
            raw["truth_density"][composition, episode],
            raw["posterior_density"][record],
            raw["integration_weights"],
        )
        nrmse_values[record] = nrmse(
            raw["high_truth"][composition, episode, free],
            raw["posterior_mean"][record, free],
        )
        nlpd_values[record] = negative_log_predictive_density(
            raw["high_truth"][composition, episode, free],
            raw["posterior_mean"][record, free],
            raw["posterior_variance"][record, free],
            raw["integration_weights"][free],
        )
        calibration_values[record] = calibration_95(
            raw["high_truth"][composition, episode, free],
            raw["posterior_mean"][record, free],
            raw["posterior_variance"][record, free],
        )
        if progress_callback is not None:
            progress_callback(
                record + 1,
                record_count,
                f"posterior metrics {record + 1}/{record_count}",
            )

    records = {
        "kl": kl_values,
        "nrmse": nrmse_values,
        "nlpd": nlpd_values,
        "calibration_95": calibration_values,
    }
    final = {
        metric: np.full((composition_count, episode_count), np.nan, dtype=float)
        for metric in records
    }
    time_average = {
        metric: np.full((composition_count, episode_count), np.nan, dtype=float)
        for metric in records
    }
    final_low_samples = np.zeros((composition_count, episode_count), dtype=np.int64)
    final_high_samples = np.zeros((composition_count, episode_count), dtype=np.int64)
    for composition in range(composition_count):
        for episode in range(episode_count):
            selected = np.flatnonzero(
                (record_composition == composition) & (record_episode == episode)
            )
            if selected.size == 0:
                raise ValueError(
                    f"composition {composition} episode {episode} has no posterior"
                )
            order = selected[np.argsort(record_time[selected])]
            times = record_time[order]
            for metric, values in records.items():
                series = values[order]
                final[metric][composition, episode] = series[-1]
                time_average[metric][composition, episode] = _time_average(
                    times, series, end_time=float(state_times[-1])
                )
            final_record = order[-1]
            final_low_samples[composition, episode] = raw["posterior_low_samples"][
                final_record
            ]
            final_high_samples[composition, episode] = raw["posterior_high_samples"][
                final_record
            ]

    total_retained = final_low_samples + final_high_samples
    if np.any(total_retained > int(raw_metadata["total_retained_samples"])):
        raise ValueError("retained sample counts exceed the fixed total budget")
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
        "scenario": str(raw_metadata.get("scenario", "default")),
        "scenario_parameters": raw_metadata.get(
            "scenario_parameters",
            {
                "num_steps": int(raw_metadata["num_steps"]),
                "num_obstacles": None,
                "obstacles_radius": None,
            },
        ),
        "compositions": [str(value) for value in names.tolist()],
        "total_robots": int(raw_metadata["total_robots"]),
        "total_retained_samples": int(raw_metadata["total_retained_samples"]),
        "episodes": episode_count,
        "num_steps": int(raw_metadata["num_steps"]),
        "dt": float(raw_metadata["dt"]),
        "initialization": raw_metadata.get(
            "initialization", {"position_policy": "uniform"}
        ),
        "timing_fields": raw_metadata["timing_fields"],
        "timing_statistics": ["median", "p95", "maximum"],
        "hyperparameter_optimization": raw_metadata.get(
            "hyperparameter_optimization", {"enabled": False}
        ),
        "aerial_ergodic_control": raw_metadata.get(
            "aerial_ergodic_control",
            {
                "mode": "centralized",
                "neighbor_weighting": "hard",
                "communication_range": None,
            },
        ),
        "metric_definitions": {
            "kl": "weighted D_KL(truth_density || posterior_density)",
            "nrmse": (
                "free-space RMSE(max(high_mean, 0), high_truth) / range(high_truth)"
            ),
            "calibration_95": ("free-space fraction inside raw GP mean +/- 1.96 std"),
            "nlpd": (
                "free-space integration-weighted marginal Gaussian negative log "
                "predictive density of latent HIGH truth using raw GP mean and "
                "variance with variance floor 1e-12"
            ),
            "time_average": (
                "trapezoidal time average with the latest posterior held to "
                "the fixed mission horizon"
            ),
        },
    }
    arrays = {
        "composition_names": names,
        "aerial_counts": raw["aerial_counts"],
        "ground_counts": raw["ground_counts"],
        "retention_low_caps": raw["retention_low_caps"],
        "retention_high_caps": raw["retention_high_caps"],
        "seeds": seeds,
        "state_times": state_times,
        "posterior_composition": record_composition,
        "posterior_episode": record_episode,
        "posterior_step": raw["posterior_step"],
        "posterior_time": record_time,
        "posterior_version": raw["posterior_version"],
        "kl": kl_values,
        "nrmse": nrmse_values,
        "nlpd": nlpd_values,
        "calibration_95": calibration_values,
        "final_kl": final["kl"],
        "final_nrmse": final["nrmse"],
        "final_nlpd": final["nlpd"],
        "final_calibration_95": final["calibration_95"],
        "time_average_kl": time_average["kl"],
        "time_average_nrmse": time_average["nrmse"],
        "time_average_nlpd": time_average["nlpd"],
        "time_average_calibration_95": time_average["calibration_95"],
        "low_submitted": raw["low_submitted"],
        "high_submitted": raw["high_submitted"],
        "total_low_submitted": np.sum(raw["low_submitted"], axis=2),
        "total_high_submitted": np.sum(raw["high_submitted"], axis=2),
        "final_low_samples": final_low_samples,
        "final_high_samples": final_high_samples,
        "final_total_samples": total_retained,
        "timing_percentiles": timing_percentiles,
    }
    arrays.update(
        {name: raw[name] for name in HYPERPARAMETER_RECORD_FIELDS if name in raw}
    )
    return save_archive(
        output_path, kind=EVALUATED_ARCHIVE_KIND, metadata=metadata, arrays=arrays
    )


def _validate_raw_shapes(
    raw: dict[str, np.ndarray],
    composition_count: int,
    episode_count: int,
    record_count: int,
) -> None:
    for name in (
        "posterior_episode",
        "posterior_step",
        "posterior_time",
        "posterior_version",
        "posterior_low_samples",
        "posterior_high_samples",
    ):
        if raw[name].shape != (record_count,):
            raise ValueError(f"{name} must match posterior record count")
    for name in HYPERPARAMETER_RECORD_FIELDS:
        if name in raw and raw[name].shape != (record_count,):
            raise ValueError(f"{name} must match posterior record count")
    expected_prefix = (composition_count, episode_count)
    for name in ("high_truth", "truth_density", "free_mask"):
        if raw[name].shape[:2] != expected_prefix:
            raise ValueError(f"{name} must match compositions and episodes")
    if np.any(raw["posterior_composition"] < 0) or np.any(
        raw["posterior_composition"] >= composition_count
    ):
        raise ValueError("posterior_composition contains an invalid index")
    if np.any(raw["posterior_episode"] < 0) or np.any(
        raw["posterior_episode"] >= episode_count
    ):
        raise ValueError("posterior_episode contains an invalid index")


def _time_average(times: np.ndarray, values: np.ndarray, *, end_time: float) -> float:
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
        path = evaluate_composition_archive(
            arguments.input,
            arguments.output,
            progress_callback=None if progress is None else progress.update,
        )
    finally:
        if progress is not None:
            progress.close()
    print(f"composition_evaluated={path}")


if __name__ == "__main__":
    main()
