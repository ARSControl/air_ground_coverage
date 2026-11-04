"""Recorded mission summary uses matching simulated HIGH truth."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest
import yaml

from docker.plot_readiness import render_summary
from src.core.deployment_scenario import build_scenario


def inputs():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    scenario = build_scenario(manifest)
    return (
        manifest,
        scenario,
        dict(
            scenario_hash=scenario.scenario_hash,
            timestamp=1.0,
            ready=True,
            vehicles={},
            command_stats={},
        ),
    )


def test_truth_panel_matches_implemented_field_and_replaces_proposals():
    manifest, scenario, report = inputs()
    fig = render_summary(report, [], manifest)
    try:
        axes = {ax.get_title(): ax for ax in fig.axes}
        truth = axes["Ground-truth HIGH field"]
        np.testing.assert_array_equal(
            np.asarray(truth.collections[0].get_array()).ravel(),
            scenario.high_field.T.ravel(),
        )
        assert not any("proposal" in title.lower() for title in axes)
        assert not scenario.high_field.flags.writeable
    finally:
        plt.close(fig)


def test_mismatched_scenario_is_rejected():
    manifest, _, report = inputs()
    manifest["field"]["components"][0]["amplitude"] += 0.1
    with pytest.raises(ValueError, match="scenario"):
        render_summary(report, [], manifest)
