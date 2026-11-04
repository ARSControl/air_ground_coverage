"""Scenario loading contracts for the simulated deployment field."""

from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
import yaml

from src.core.deployment_scenario import build_scenario


def config():
    return yaml.safe_load(Path("docker/scenarios/example_sitl.yaml").read_text())


def test_example_resolves_deterministically_and_samples():
    first = build_scenario(config())
    second = build_scenario(config())
    assert first.scenario_hash == second.scenario_hash
    np.testing.assert_array_equal(first.obstacle_map, second.obstacle_map)
    assert first.obstacle_map[25, 25]
    first_batch = first.service.sample(
        first.scenario_hash, "uav_1", "1", 1, (10, 10), 0
    )
    second_batch = second.service.sample(
        second.scenario_hash, "uav_1", "1", 1, (10, 10), 0
    )
    assert first_batch == second_batch and first_batch
    assert first_batch[0].robot_id == "uav_1"


def test_manifest_hash_changes_and_input_is_not_mutated():
    source = config()
    original = deepcopy(source)
    first = build_scenario(source)
    assert source == original
    source["field"]["components"][0]["amplitude"] = 2
    assert build_scenario(source).scenario_hash != first.scenario_hash


@pytest.mark.parametrize("change", ["duplicate", "role", "resolution", "unknown"])
def test_invalid_manifest_is_rejected(change):
    data = config()
    if change == "duplicate":
        data["vehicles"][1]["id"] = data["vehicles"][0]["id"]
    elif change == "role":
        data["vehicles"][0]["sensor"]["fidelity"] = "low"
    elif change == "resolution":
        data["map"]["resolution_m"] = 0
    else:
        data["field"]["kind"] = "unknown"
    with pytest.raises(ValueError):
        build_scenario(data)
