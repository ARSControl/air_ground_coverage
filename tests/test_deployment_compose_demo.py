"""Deterministic smoke runner for the containerized deployment core."""

from docker.deployment_smoke import run_smoke


def test_smoke_exercises_both_fidelities_and_changes_target():
    result = run_smoke("docker/scenarios/example_sitl.yaml")
    assert result["low_count"] > 0
    assert result["high_count"] > 0
    assert result["posterior_version"] == 2
    assert result["target_max_abs_change"] > 0
