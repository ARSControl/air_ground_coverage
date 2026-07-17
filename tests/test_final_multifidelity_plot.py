"""Tests for the opt-in final multi-fidelity state plot."""

from __future__ import annotations

from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
import pytest

from examples.plot_final_multifidelity_state import (
    _ground_importance_grid,
    _plot_combined_trajectories,
    render_final_multifidelity_state,
)
from examples.run_multifidelity import save_final_plot_if_requested
from src.core.base import HEDACParams


def _snapshot():
    grid_x, grid_y = np.meshgrid(np.linspace(0.0, 2.0, 5), np.linspace(0.0, 2.0, 5))
    points = np.column_stack((grid_x.ravel(), grid_y.ravel()))
    mean = np.sin(points[:, 0]) + np.cos(points[:, 1])
    variance = 0.05 + 0.1 * points[:, 0]
    density = np.exp(mean)
    density /= np.sum(density)
    return SimpleNamespace(
        query_points=points,
        query_shape=grid_x.shape,
        high_mean=mean,
        high_variance=variance,
        density=density,
        version=3,
        low_sample_count=9,
        high_sample_count=4,
    )


def _result():
    return SimpleNamespace(
        aerial_trajectories=(
            np.array([[0.2, 0.2], [0.7, 0.6], [1.2, 1.1]]),
        ),
        ground_trajectories=(
            np.array([[1.8, 0.2], [1.5, 0.5], [1.2, 0.9]]),
        ),
    )


def _truth():
    x_values, y_values = np.meshgrid(
        np.linspace(0.0, 2.0, 7), np.linspace(0.0, 2.0, 6)
    )
    return np.exp(-((x_values - 1.4) ** 2 + (y_values - 0.8) ** 2))


def test_renderer_saves_nonempty_png(tmp_path) -> None:
    output_path = tmp_path / "nested" / "final.png"
    returned = render_final_multifidelity_state(
        _result(), _snapshot(), _truth(), output_path
    )
    assert returned == output_path
    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_path.stat().st_size > 10_000
    image = plt.imread(output_path)
    assert image.ndim == 3
    assert image.shape[0] > 500
    assert image.shape[1] > 500


def test_ground_importance_panel_uses_exact_published_density() -> None:
    snapshot = _snapshot()
    plotted = _ground_importance_grid(snapshot)
    np.testing.assert_array_equal(
        plotted,
        snapshot.density.reshape(snapshot.query_shape),
    )
    assert not np.array_equal(
        plotted,
        snapshot.high_mean.reshape(snapshot.query_shape),
    )


def test_combined_trajectory_overlay_uses_one_color_per_robot_class() -> None:
    figure, axis = plt.subplots()
    aerial = (
        np.array([[0.0, 0.0], [1.0, 1.0]]),
        np.array([[0.0, 1.0], [1.0, 2.0]]),
    )
    ground = (
        np.array([[2.0, 0.0], [1.5, 0.5]]),
        np.array([[2.0, 2.0], [1.5, 1.5]]),
    )
    _plot_combined_trajectories(axis, aerial, ground)
    assert [line.get_color() for line in axis.lines] == [
        "#2563eb",
        "#2563eb",
        "#f97316",
        "#f97316",
    ]
    labels = axis.get_legend_handles_labels()[1]
    assert labels == [
        "Aerial trajectories",
        "Aerial final states",
        "Ground trajectories",
        "Ground final states",
    ]
    plt.close(figure)


def test_configured_plot_is_saved_and_reported(tmp_path) -> None:
    output_path = tmp_path / "configured.png"
    params = HEDACParams.from_dict(
        {
            "visualization": {
                "save_final_plot": True,
                "final_plot_path": str(output_path),
            }
        }
    )
    messages: list[str] = []
    simulation = SimpleNamespace(
        latest_posterior=_snapshot(),
        hedac=SimpleNamespace(goal_density=_truth()),
    )
    returned = save_final_plot_if_requested(
        params,
        simulation,
        _result(),
        plots_disabled=False,
        emit=messages.append,
    )
    assert returned == output_path
    assert output_path.is_file()
    assert messages == [f"final_plot={output_path}"]


@pytest.mark.parametrize(
    ("configured", "plots_disabled"),
    [(False, False), (True, True)],
)
def test_disabled_plot_does_not_create_output(
    tmp_path, configured: bool, plots_disabled: bool
) -> None:
    output_path = tmp_path / "disabled.png"
    params = HEDACParams.from_dict(
        {
            "visualization": {
                "save_final_plot": configured,
                "final_plot_path": str(output_path),
            }
        }
    )
    returned = save_final_plot_if_requested(
        params,
        SimpleNamespace(latest_posterior=_snapshot()),
        _result(),
        plots_disabled=plots_disabled,
    )
    assert returned is None
    assert not output_path.exists()


def test_enabled_plot_without_posterior_is_skipped(tmp_path) -> None:
    params = HEDACParams.from_dict(
        {
            "visualization": {
                "save_final_plot": True,
                "final_plot_path": str(tmp_path / "missing.png"),
            }
        }
    )
    messages: list[str] = []
    returned = save_final_plot_if_requested(
        params,
        SimpleNamespace(latest_posterior=None),
        _result(),
        plots_disabled=False,
        emit=messages.append,
    )
    assert returned is None
    assert messages == [
        "note=final plot skipped because no valid posterior is available"
    ]
