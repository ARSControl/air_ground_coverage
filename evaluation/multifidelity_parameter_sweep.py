"""Proposed-method-only parameter sweep resolution, evaluation, and plotting."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import yaml

from evaluation.comparison_io import (
    MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND,
    load_archive,
    save_archive,
)
from evaluation.multifidelity_metrics import (
    calibration_95,
    covered_probability_mass,
    footprint_normalized_coverage,
    kl_divergence,
    maximum_density_mass_for_area,
    negative_log_predictive_density,
    nrmse,
    sector_footprint_area,
)


EVALUATED_KIND = "multifidelity_parameter_condition_evaluated"
SUPPORTED_FACTORS = (
    "environment_size",
    "team_composition",
    "duration",
    "obstacle_density",
    "sensing_range",
    "measurement_noise",
)
METRICS = (
    ("final_nrmse", "Final NRMSE", "lower"),
    ("final_kl", "Final KL divergence", "lower"),
    ("final_nlpd", "Final marginal NLPD", "lower"),
    ("final_calibration_95", "Final 95% calibration", "target 0.95"),
    ("final_coverage", "Final coverage effectiveness", "higher"),
    ("final_locational_cost", "Final locational cost", "lower"),
)


@dataclass(frozen=True)
class SweepDefinition:
    source_path: Path
    base_config_path: Path
    base_document: dict[str, Any]
    factors: dict[str, tuple[dict[str, Any], ...]]


@dataclass(frozen=True)
class SweepCondition:
    factor: str
    label: str
    slug: str
    resolved_value: dict[str, Any]
    config_path: Path


def load_sweep_definition(path: str | Path) -> SweepDefinition:
    """Load and validate a one-factor-at-a-time sweep declaration."""
    source = Path(path)
    with source.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise TypeError("parameter sweep configuration must be a mapping")
    raw_base = document.get("base_config")
    if not isinstance(raw_base, str) or not raw_base.strip():
        raise ValueError("base_config must be a nonempty path")
    base_path = Path(raw_base)
    if not base_path.is_file():
        candidate = source.parent / base_path
        if candidate.is_file():
            base_path = candidate
        else:
            raise FileNotFoundError(f"base configuration not found: {raw_base}")
    with base_path.open(encoding="utf-8") as stream:
        base_document = yaml.safe_load(stream)
    if not isinstance(base_document, dict):
        raise TypeError("base configuration must be a mapping")
    sweep = document.get("parameter_sweep")
    if not isinstance(sweep, dict) or not isinstance(sweep.get("factors"), dict):
        raise ValueError("parameter_sweep.factors must be a mapping")
    raw_factors = sweep["factors"]
    unknown = set(raw_factors).difference(SUPPORTED_FACTORS)
    missing = set(SUPPORTED_FACTORS).difference(raw_factors)
    if unknown or missing:
        raise ValueError(
            "parameter sweep factors must be exactly "
            f"{SUPPORTED_FACTORS}; missing={sorted(missing)}, unknown={sorted(unknown)}"
        )
    factors: dict[str, tuple[dict[str, Any], ...]] = {}
    for factor in SUPPORTED_FACTORS:
        levels = raw_factors[factor]
        if not isinstance(levels, list) or len(levels) < 2:
            raise ValueError(f"factor {factor!r} must declare at least two levels")
        copied: list[dict[str, Any]] = []
        labels: set[str] = set()
        for level in levels:
            if not isinstance(level, dict):
                raise TypeError(f"levels for {factor!r} must be mappings")
            label = level.get("label")
            if not isinstance(label, str) or not label.strip():
                raise ValueError(f"every {factor!r} level needs a nonempty label")
            if label in labels:
                raise ValueError(f"duplicate label {label!r} in factor {factor!r}")
            labels.add(label)
            copied.append(deepcopy(level))
        references = [level for level in copied if level.get("reference") is True]
        if len(references) != 1:
            raise ValueError(
                f"factor {factor!r} must declare exactly one reference level"
            )
        factors[factor] = tuple(copied)
    return SweepDefinition(source, base_path, base_document, factors)


def resolve_conditions(
    definition: SweepDefinition, config_directory: str | Path
) -> tuple[SweepCondition, ...]:
    """Write one resolved YAML per level while changing only its factor."""
    destination = Path(config_directory)
    destination.mkdir(parents=True, exist_ok=True)
    reference_document = deepcopy(definition.base_document)
    for factor in SUPPORTED_FACTORS:
        reference_level = next(
            level
            for level in definition.factors[factor]
            if level.get("reference") is True
        )
        _apply_level(reference_document, factor, reference_level)
    conditions: list[SweepCondition] = []
    for factor in SUPPORTED_FACTORS:
        for level in definition.factors[factor]:
            document = deepcopy(reference_document)
            resolved = _apply_level(document, factor, level)
            label = str(level["label"])
            slug = f"{_safe_name(factor)}__{_safe_name(label)}"
            document["parameter_condition"] = {
                "factor": factor,
                "label": label,
                "resolved_value": resolved,
                "source_sweep": str(definition.source_path),
                "base_config": str(definition.base_config_path),
                "baselines_included": False,
            }
            config_path = destination / f"{slug}.yaml"
            _atomic_yaml(config_path, document)
            conditions.append(
                SweepCondition(factor, label, slug, resolved, config_path)
            )
    return tuple(conditions)


def evaluate_parameter_condition(raw_path: str | Path, output_path: str | Path) -> Path:
    """Evaluate one saved proposed-method condition without rerunning it."""
    raw_metadata, raw = load_archive(
        raw_path, expected_kind=MULTIFIDELITY_COMPARISON_RAW_ARCHIVE_KIND
    )
    if str(raw["method_name"].item()) != "multifidelity":
        raise ValueError("parameter sweep accepts only multifidelity raw archives")
    seeds = raw["seeds"]
    episodes = seeds.size
    record_episode = raw["posterior_episode"].astype(int)
    record_time = raw["posterior_time"]
    record_count = record_episode.size
    metric_records = {
        "nrmse": np.empty(record_count),
        "kl": np.empty(record_count),
        "nlpd": np.empty(record_count),
        "calibration_95": np.empty(record_count),
    }
    weights = raw["integration_weights"]
    for record in range(record_count):
        episode = record_episode[record]
        free = raw["free_mask"][episode].astype(bool)
        truth = raw["high_truth"][episode]
        metric_records["nrmse"][record] = nrmse(
            truth[free], raw["posterior_mean"][record, free]
        )
        metric_records["kl"][record] = kl_divergence(
            raw["truth_density"][episode], raw["posterior_density"][record], weights
        )
        metric_records["nlpd"][record] = negative_log_predictive_density(
            truth[free],
            raw["posterior_mean"][record, free],
            raw["posterior_variance"][record, free],
            weights[free],
        )
        metric_records["calibration_95"][record] = calibration_95(
            truth[free],
            raw["posterior_mean"][record, free],
            raw["posterior_variance"][record, free],
        )
    end_time = float(raw["state_times"][-1])
    summaries: dict[str, np.ndarray] = {}
    for name, values in metric_records.items():
        final = np.full(episodes, np.nan)
        average = np.full(episodes, np.nan)
        for episode in range(episodes):
            selected = np.flatnonzero(record_episode == episode)
            if selected.size:
                order = selected[np.argsort(record_time[selected])]
                final[episode] = values[order[-1]]
                average[episode] = _held_time_average(
                    record_time[order], values[order], end_time
                )
        summaries[f"final_{name}"] = final
        summaries[f"time_average_{name}"] = average

    times = raw["state_times"]
    states = raw["ground_states"]
    coverage = np.empty((episodes, times.size), dtype=float)
    locational_cost = np.empty_like(coverage)
    ground_range = float(raw["ground_sensing_range"])
    ground_fov = float(raw["ground_fov_degrees"])
    for episode in range(episodes):
        free = raw["free_mask"][episode].astype(bool)
        free_area = float(np.sum(weights[free]))
        nominal_area = states.shape[2] * sector_footprint_area(
            fov_degrees=ground_fov, sensing_range=ground_range
        )
        oracle = maximum_density_mass_for_area(
            raw["truth_density"][episode],
            weights,
            area_budget=min(nominal_area, free_area),
            free_mask=free,
        )
        for state_index, state in enumerate(states[episode]):
            mass = covered_probability_mass(
                raw["query_points"],
                raw["truth_density"][episode],
                weights,
                state,
                fov_degrees=ground_fov,
                sensing_range=ground_range,
            )
            coverage[episode, state_index] = footprint_normalized_coverage(mass, oracle)
            squared = np.sum(
                (raw["query_points"][:, None, :] - state[None, :, :2]) ** 2,
                axis=2,
            )
            locational_cost[episode, state_index] = float(
                np.sum(
                    np.min(squared, axis=1) * raw["truth_density"][episode] * weights
                )
            )
    duration = times[-1] - times[0]
    summaries.update(
        {
            "final_coverage": coverage[:, -1],
            "time_average_coverage": np.trapezoid(coverage, times, axis=1) / duration,
            "final_locational_cost": locational_cost[:, -1],
            "time_average_locational_cost": np.trapezoid(locational_cost, times, axis=1)
            / duration,
        }
    )
    condition = raw_metadata["resolved_aerial_config"].get("parameter_condition")
    metadata = {
        "method": "multifidelity",
        "baselines_included": False,
        "raw_source": str(Path(raw_path)),
        "condition": condition,
        "episodes": int(episodes),
        "dt": float(raw_metadata["dt"]),
    }
    arrays = {
        "seeds": seeds,
        "state_times": times,
        "posterior_episode": record_episode,
        "posterior_time": record_time,
        **metric_records,
        "coverage": coverage,
        "locational_cost": locational_cost,
        **summaries,
    }
    return save_archive(
        output_path, kind=EVALUATED_KIND, metadata=metadata, arrays=arrays
    )


def plot_parameter_factor(
    factor: str,
    conditions: list[dict[str, Any]],
    root: str | Path,
    output_path: str | Path,
    *,
    bootstrap_samples: int,
) -> Path:
    """Plot episode-level final metrics for all levels of one factor."""
    if bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be positive")
    labels: list[str] = []
    loaded: list[dict[str, np.ndarray]] = []
    for condition in conditions:
        labels.append(str(condition["label"]))
        _, values = load_archive(
            Path(root) / condition["evaluated_archive"], expected_kind=EVALUATED_KIND
        )
        loaded.append(values)
    figure, axes = plt.subplots(2, 3, figsize=(13.0, 7.2), constrained_layout=True)
    rng = np.random.default_rng(20261021)
    x_values = np.arange(len(labels), dtype=float)
    for axis, (metric, title, direction) in zip(axes.ravel(), METRICS, strict=True):
        means: list[float] = []
        lower: list[float] = []
        upper: list[float] = []
        for index, values in enumerate(loaded):
            samples = np.asarray(values[metric], dtype=float)
            samples = samples[np.isfinite(samples)]
            if not samples.size:
                means.append(np.nan)
                lower.append(np.nan)
                upper.append(np.nan)
                continue
            means.append(float(np.mean(samples)))
            selections = rng.integers(
                0, samples.size, size=(bootstrap_samples, samples.size)
            )
            boot = np.mean(samples[selections], axis=1)
            bounds = np.percentile(boot, (2.5, 97.5))
            lower.append(float(bounds[0]))
            upper.append(float(bounds[1]))
            axis.scatter(
                np.full(samples.size, index), samples, color="#777777", s=13, alpha=0.5
            )
        means_array = np.asarray(means)
        axis.errorbar(
            x_values,
            means_array,
            yerr=np.vstack((means_array - lower, upper - means_array)),
            color="#0072B2",
            marker="o",
            linewidth=1.8,
            capsize=4,
        )
        if metric == "final_calibration_95":
            axis.axhline(0.95, color="#333333", linestyle="--", linewidth=1.0)
        axis.set_title(f"{title} ({direction})")
        axis.set_xticks(x_values, labels, rotation=20, ha="right")
        axis.grid(True, alpha=0.25)
    figure.suptitle(
        f"Proposed MFGP sensitivity: {factor.replace('_', ' ')}\n"
        "points: episodes; marker/error bar: mean and bootstrap 95% interval"
    )
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def plot_parameter_metric_overview(
    conditions: list[dict[str, Any]],
    root: str | Path,
    output_path: str | Path,
    *,
    metric: str = "final_kl",
    bootstrap_samples: int = 2000,
) -> Path:
    """Plot one metric for every factor in a compact small-multiple figure."""
    if bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be positive")
    grouped = _group_metric_conditions(conditions, root, metric)
    if not grouped:
        raise ValueError("no parameter-sweep conditions were supplied")
    columns = 3
    rows = math.ceil(len(grouped) / columns)
    figure, axes = plt.subplots(
        rows,
        columns,
        figsize=(13.0, 3.6 * rows),
        sharey=True,
        squeeze=False,
        constrained_layout=True,
    )
    rng = np.random.default_rng(20261021)
    for axis, (factor, entries) in zip(axes.ravel(), grouped.items()):
        labels: list[str] = []
        for index, (condition, values) in enumerate(entries):
            samples = values[metric]
            labels.append(str(condition["label"]))
            axis.scatter(
                np.full(samples.size, index),
                samples,
                color="#777777",
                s=13,
                alpha=0.45,
                zorder=2,
            )
            mean, lower, upper = _bootstrap_mean_interval(
                samples, rng, bootstrap_samples
            )
            axis.errorbar(
                index,
                mean,
                yerr=[[mean - lower], [upper - mean]],
                color="#0072B2",
                marker="o",
                capsize=4,
                linewidth=1.8,
                zorder=3,
            )
        axis.set_title(factor.replace("_", " ").title())
        axis.set_xticks(range(len(labels)), labels, rotation=20, ha="right")
        axis.grid(True, axis="y", alpha=0.25)
    for axis in axes.ravel()[len(grouped) :]:
        axis.set_visible(False)
    for axis in axes[:, 0]:
        axis.set_ylabel(_metric_title(metric))
    figure.suptitle(f"Proposed MFGP parameter sensitivity: {_metric_title(metric)}")
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def plot_parameter_metric_effects(
    conditions: list[dict[str, Any]],
    root: str | Path,
    output_path: str | Path,
    *,
    metric: str = "final_kl",
    bootstrap_samples: int = 2000,
) -> Path:
    """Plot paired changes from each factor's reference condition."""
    if bootstrap_samples < 1:
        raise ValueError("bootstrap_samples must be positive")
    grouped = _group_metric_conditions(conditions, root, metric)
    rng = np.random.default_rng(20261021)
    labels: list[str] = []
    means: list[float] = []
    lower: list[float] = []
    upper: list[float] = []
    for factor, entries in grouped.items():
        references = [entry for entry in entries if _is_reference(entry[0])]
        if len(references) != 1:
            raise ValueError(f"factor {factor!r} must have exactly one reference")
        reference_condition, reference_values = references[0]
        reference_by_seed = dict(
            zip(reference_values["seeds"].tolist(), reference_values[metric], strict=True)
        )
        for condition, values in entries:
            if condition is reference_condition:
                continue
            paired: list[float] = []
            for seed, value in zip(values["seeds"], values[metric], strict=True):
                if seed.item() in reference_by_seed:
                    paired.append(float(value - reference_by_seed[seed.item()]))
            if not paired:
                raise ValueError(
                    f"condition {condition['label']!r} has no seeds paired with its reference"
                )
            samples = np.asarray(paired, dtype=float)
            mean, low, high = _bootstrap_mean_interval(
                samples, rng, bootstrap_samples
            )
            labels.append(f"{factor.replace('_', ' ')}: {condition['label']}")
            means.append(mean)
            lower.append(low)
            upper.append(high)
    y = np.arange(len(labels), dtype=float)
    means_array = np.asarray(means)
    figure_height = max(5.0, 0.42 * len(labels) + 1.8)
    figure, axis = plt.subplots(figsize=(10.5, figure_height), constrained_layout=True)
    axis.errorbar(
        means_array,
        y,
        xerr=np.vstack((means_array - lower, np.asarray(upper) - means_array)),
        color="#0072B2",
        marker="o",
        linestyle="none",
        capsize=4,
    )
    axis.axvline(0.0, color="#333333", linestyle="--", linewidth=1.0)
    axis.set_yticks(y, labels)
    axis.invert_yaxis()
    axis.set_xlabel(f"Paired change from factor reference in {_metric_title(metric)}")
    axis.set_title("Proposed MFGP parameter effects (negative is lower)")
    axis.grid(True, axis="x", alpha=0.25)
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=180, facecolor="white", bbox_inches="tight")
    plt.close(figure)
    return destination


def _group_metric_conditions(
    conditions: list[dict[str, Any]], root: str | Path, metric: str
) -> dict[str, list[tuple[dict[str, Any], dict[str, np.ndarray]]]]:
    grouped: dict[str, list[tuple[dict[str, Any], dict[str, np.ndarray]]]] = {}
    for condition in conditions:
        _, values = load_archive(
            Path(root) / condition["evaluated_archive"], expected_kind=EVALUATED_KIND
        )
        if metric not in values:
            raise ValueError(f"metric {metric!r} is not present in evaluated archives")
        samples = np.asarray(values[metric], dtype=float)
        finite = np.isfinite(samples)
        if not np.all(finite):
            values = dict(values)
            values[metric] = samples[finite]
            values["seeds"] = np.asarray(values["seeds"])[finite]
        grouped.setdefault(str(condition["factor"]), []).append((condition, values))
    return grouped


def _is_reference(condition: dict[str, Any]) -> bool:
    if "reference" in condition:
        return condition["reference"] is True
    label = str(condition.get("label", "")).lower()
    return label == "reference" or label.startswith("reference_")


def _bootstrap_mean_interval(
    samples: np.ndarray, rng: np.random.Generator, bootstrap_samples: int
) -> tuple[float, float, float]:
    samples = np.asarray(samples, dtype=float)
    samples = samples[np.isfinite(samples)]
    if not samples.size:
        raise ValueError("cannot summarize an empty metric sample")
    selections = rng.integers(0, samples.size, size=(bootstrap_samples, samples.size))
    boot = np.mean(samples[selections], axis=1)
    low, high = np.percentile(boot, (2.5, 97.5))
    return float(np.mean(samples)), float(low), float(high)


def _metric_title(metric: str) -> str:
    titles = {name: title for name, title, _ in METRICS}
    return titles.get(metric, metric.replace("_", " ").title())


def write_manifest(path: str | Path, document: dict[str, Any]) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(document, indent=2, sort_keys=True) + "\n"
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(encoded)
        os.replace(temporary_name, destination)
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)
    return destination


def _apply_level(
    document: dict[str, Any], factor: str, level: dict[str, Any]
) -> dict[str, Any]:
    if factor == "environment_size":
        size = _integer_pair(level.get("size"), "size")
        points = _positive_integer(
            level.get("ground_local_grid_points"), "ground_local_grid_points"
        )
        _set(document, "map.size", list(size))
        _set(document, "ground.map.local_grid_points", points)
        return {"size": list(size), "ground_local_grid_points": points}
    if factor == "team_composition":
        aerial = _nonnegative_integer(level.get("aerial"), "aerial")
        ground = _nonnegative_integer(level.get("ground"), "ground")
        if aerial < 1 or ground < 1:
            raise ValueError("parameter sweep compositions require both robot classes")
        _set(document, "aerial.simulation.num_agents", aerial)
        _set(document, "ground.simulation.num_agents", ground)
        budget = _positive_integer(
            document.get("composition_sweep", {}).get(
                "total_retained_samples",
                document["multifidelity"]["retention"]["max_low_samples"]
                + document["multifidelity"]["retention"]["max_high_samples"],
            ),
            "total_retained_samples",
        )
        low = max(1, int(round(budget * aerial / (aerial + ground))))
        high = max(1, budget - low)
        _set(document, "multifidelity.retention.max_low_samples", low)
        _set(document, "multifidelity.retention.max_high_samples", high)
        return {
            "aerial": aerial,
            "ground": ground,
            "retention_low": low,
            "retention_high": high,
        }
    if factor == "duration":
        steps = _positive_integer(level.get("num_steps"), "num_steps")
        _set(document, "simulation.num_steps", steps)
        return {
            "num_steps": steps,
            "duration_seconds": steps * document["simulation"]["dt"],
        }
    if factor == "obstacle_density":
        count = _nonnegative_integer(level.get("count"), "count")
        radius = _positive_float(level.get("radius"), "radius")
        _set(document, "ground.simulation.num_obstacles", count)
        _set(document, "ground.simulation.obstacles_radius", radius)
        size = document["map"]["size"]
        occupied_fraction = count * math.pi * radius**2 / (size[0] * size[1])
        return {
            "count": count,
            "radius": radius,
            "nominal_area_fraction": occupied_fraction,
        }
    if factor == "sensing_range":
        if level.get("full_environment") is True:
            size = document["map"]["size"]
            aerial = ground = float(math.hypot(size[0], size[1]))
            full = True
        else:
            aerial = _positive_float(level.get("aerial"), "aerial")
            ground = _positive_float(level.get("ground"), "ground")
            full = False
        _set(document, "aerial.sensor.fov_depth", aerial)
        _set(document, "ground.sensor.fov_depth", ground)
        return {
            "aerial": aerial,
            "ground": ground,
            "covers_full_environment": full,
        }
    if factor == "measurement_noise":
        low = _nonnegative_float(level.get("low_variance"), "low_variance")
        high = _nonnegative_float(level.get("high_variance"), "high_variance")
        _set(document, "multifidelity.low_noise_variance", low)
        _set(document, "multifidelity.high_noise_variance", high)
        return {"low_variance": low, "high_variance": high, "matched": True}
    raise ValueError(f"unsupported factor: {factor}")


def _set(document: dict[str, Any], dotted: str, value: Any) -> None:
    current = document
    for component in dotted.split(".")[:-1]:
        current = current.setdefault(component, {})
        if not isinstance(current, dict):
            raise TypeError(f"cannot set {dotted!r} through a non-mapping")
    current[dotted.split(".")[-1]] = value


def _held_time_average(times: np.ndarray, values: np.ndarray, end: float) -> float:
    if end > times[-1]:
        times = np.append(times, end)
        values = np.append(values, values[-1])
    if times.size == 1 or times[-1] == times[0]:
        return float(values[-1])
    return float(np.trapezoid(values, times) / (times[-1] - times[0]))


def _safe_name(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_").lower()
    if not result:
        raise ValueError("labels must contain a filename-safe character")
    return result


def _atomic_yaml(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = yaml.safe_dump(document, sort_keys=False)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary_name = stream.name
            stream.write(encoded)
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def _integer_pair(value: Any, name: str) -> tuple[int, int]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{name} must contain two integers")
    return (_positive_integer(value[0], name), _positive_integer(value[1], name))


def _positive_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _positive_float(value: Any, name: str) -> float:
    result = _nonnegative_float(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be positive")
    return result


def _nonnegative_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a real scalar")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return result
