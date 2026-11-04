"""Run the current deployment core once without connecting to vehicles."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from src.core.deployment_estimator import DeploymentEstimator
from src.core.deployment_scenario import build_scenario
from src.core.multifidelity_estimator import UpdateStatus


def run_smoke(scenario_path):
    """Exercise LOW and HIGH samples and verify target feedback offline."""
    with Path(scenario_path).open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    scenario = build_scenario(manifest)
    estimator = DeploymentEstimator(manifest, scenario.scenario_hash, grid_points=21)
    vehicles = manifest["vehicles"]
    high_vehicle = next(v for v in vehicles if v["role"] == "ground")
    low_vehicle = next(v for v in vehicles if v["role"] == "aerial")
    grid = manifest["map"]
    origin = np.asarray(grid["origin_m"], dtype=float)
    rows, cols = grid["shape_rows_columns"]
    position = origin + float(grid["resolution_m"]) * np.array(
        [(cols - 1) * 0.3, (rows - 1) * 0.3]
    )
    low = scenario.service.sample(
        scenario.scenario_hash, low_vehicle["id"], "low", 0, position, 0
    )
    if not low:
        raise RuntimeError("smoke scenario produced no LOW observations")
    estimator.submit_batch(scenario.scenario_hash, low_vehicle["id"], "low", low)
    if estimator.update(0).status is not UpdateStatus.UPDATED:
        raise RuntimeError("LOW estimator update failed")
    before = estimator.aerial_target()
    high = scenario.service.sample(
        scenario.scenario_hash, high_vehicle["id"], "high", 0.5, position, 0
    )
    if not high:
        raise RuntimeError("smoke scenario produced no HIGH observations")
    estimator.submit_batch(scenario.scenario_hash, high_vehicle["id"], "high", high)
    report = estimator.update(max(0.5, estimator.period))
    if report is None or report.status is not UpdateStatus.UPDATED:
        raise RuntimeError("HIGH estimator update failed")
    change = float(np.max(np.abs(estimator.aerial_target() - before)))
    if change <= 0:
        raise RuntimeError("HIGH observation did not change aerial target")
    return {
        "scenario_hash": scenario.scenario_hash,
        "low_count": len(low),
        "high_count": len(high),
        "posterior_version": report.version,
        "target_max_abs_change": change,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="docker/scenarios/example_sitl.yaml")
    args = parser.parse_args()
    print(json.dumps(run_smoke(args.scenario), sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
