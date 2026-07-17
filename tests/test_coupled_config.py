"""Tests for the self-contained coupled aerial/ground YAML schema."""

from __future__ import annotations

from pathlib import Path

import pytest

from src.coupled_config import load_coupled_configuration


def test_canonical_config_resolves_shared_and_robot_specific_values() -> None:
    resolved = load_coupled_configuration("configs/multifidelity.yaml")

    assert resolved.path == Path("configs/multifidelity.yaml")
    assert resolved.aerial.get("estimator_mode") == "multifidelity"
    assert resolved.ground.get("estimator_mode") == "multifidelity"
    assert resolved.aerial.num_steps == resolved.ground.num_steps == 300
    assert resolved.aerial.dt == resolved.ground.dt == 0.1
    assert resolved.aerial.random_seed == resolved.ground.random_seed == 42
    assert resolved.aerial.map_config["size"] == [50, 50]
    assert resolved.ground.map_config["size"] == [50, 50]
    assert resolved.aerial.resolution == 1.0
    assert resolved.ground.resolution == 0.1
    assert resolved.aerial.num_agents == 3
    assert resolved.ground.num_agents == 7
    assert resolved.aerial.get("agents.model_type") == "dubins"
    assert resolved.ground.get("agents.model_type") == "unicycle"
    assert resolved.ground.get("controller.type") == "lloyd"
    assert resolved.ground.local_grid_points == 100
    assert resolved.ground.mpc_horizon == 2
    assert resolved.ground.get("lloyd.position_gain") == 1.0
    assert resolved.ground.get("lloyd.heading_gain") == 2.0
    assert resolved.ground.get("lloyd.centroid_tolerance") == 0.05
    assert resolved.ground.get("lloyd.max_linear_velocity") == 4.0
    assert resolved.ground.get("lloyd.max_angular_velocity") == 5.0
    assert resolved.aerial.get("multifidelity.rho") == 0.8
    assert resolved.ground.get("multifidelity.rho") == 0.8
    assert resolved.aerial.get(
        "multifidelity.hyperparameter_optimization.enabled"
    ) is True
    assert resolved.ground.get(
        "multifidelity.hyperparameter_optimization.fit_interval_updates"
    ) == 5
    assert resolved.aerial.get(
        "multifidelity.hyperparameter_optimization.bounds.low_length_scale"
    ) == [1.0, 20.0]
    assert resolved.aerial.get("aerial") is None
    assert resolved.aerial.get("ground") is None
    assert resolved.ground.get("aerial") is None
    assert resolved.ground.get("ground") is None


def test_nested_overrides_do_not_remove_shared_siblings(tmp_path) -> None:
    path = tmp_path / "coupled.yaml"
    path.write_text(
        """
estimator_mode: multifidelity
simulation:
  num_steps: 5
  dt: 0.2
map:
  size: [8, 9]
  resolution: 1.0
aerial:
  simulation:
    num_agents: 2
ground:
  simulation:
    num_agents: 4
  map:
    resolution: 0.25
""",
        encoding="utf-8",
    )
    resolved = load_coupled_configuration(path)
    assert resolved.aerial.num_steps == resolved.ground.num_steps == 5
    assert resolved.aerial.dt == resolved.ground.dt == 0.2
    assert resolved.aerial.map_config == {"size": [8, 9], "resolution": 1.0}
    assert resolved.ground.map_config == {"size": [8, 9], "resolution": 0.25}
    assert resolved.aerial.num_agents == 2
    assert resolved.ground.num_agents == 4


@pytest.mark.parametrize(
    ("contents", "exception", "message"),
    [
        ("- not\n- a\n- mapping\n", TypeError, "YAML mapping"),
        ("ground: {}\n", ValueError, "'aerial' section"),
        ("aerial: []\nground: {}\n", TypeError, "'aerial' section"),
    ],
)
def test_invalid_coupled_schema_is_rejected(
    tmp_path, contents: str, exception: type[Exception], message: str
) -> None:
    path = tmp_path / "invalid.yaml"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(exception, match=message):
        load_coupled_configuration(path)
