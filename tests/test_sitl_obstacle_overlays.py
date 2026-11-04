"""Obstacle overlays use East/North plotting axes and physical CBF radii."""
from pathlib import Path
import matplotlib.pyplot as plt
import yaml
from docker.plot_obstacles import draw_obstacles
from docker.plot_readiness import render_summary
from src.core.deployment_scenario import build_scenario


def manifest():
    return yaml.safe_load(Path('docker/scenarios/sitl_three_drone_two_obstacles.yaml').read_text())


def test_geometry_and_disabled_clearance():
    m = manifest()
    fig, ax = plt.subplots()
    try:
        disks = draw_obstacles(ax, m)
        assert [d.center for d in disks] == [(25,25),(25,25),(30,32),(30,32)]
        assert [d.radius for d in disks] == [3,4,3,4]
        assert disks[1].get_linestyle() == '--'
        m['control']['obstacle_cbf']['enabled'] = False
        assert len(draw_obstacles(ax, m)) == 2
        m['map']['obstacles']['circles'] = []
        assert draw_obstacles(ax, m) == []
    finally:
        plt.close(fig)


def test_summary_only_adds_overlays_to_map_panels():
    m = manifest()
    report = dict(scenario_hash=build_scenario(m).scenario_hash, timestamp=1., ready=True, vehicles={})
    fig = render_summary(report, [], m)
    try:
        maps = [fig.axes[i] for i in (0, 1, 2, 4, 5)]
        assert len(maps) == 5
        assert all(len(ax.patches) == 4 for ax in maps)
        counts = next(ax for ax in fig.axes if ax.get_title() == 'Central retained observations')
        assert not counts.patches
    finally:
        plt.close(fig)


def test_video_draws_overlays_on_all_three_panels(monkeypatch, tmp_path):
    from tests.test_replay_sitl import fixture
    from docker import replay_sitl
    report, events, m = fixture()
    called = []
    original = replay_sitl.draw_obstacles
    def track(ax, manifest):
        called.append(original(ax, manifest))
    monkeypatch.setattr(replay_sitl, 'draw_obstacles', track)
    if not replay_sitl.animation.writers.is_available('ffmpeg'):
        import pytest
        pytest.skip('ffmpeg unavailable')
    replay_sitl.render(replay_sitl.MissionReplay(report, events, m), tmp_path/'test.mp4', fps=1, speed=10, dpi=40)
    assert len(called) == 3
    assert all(len(artists) == 1 for artists in called)
