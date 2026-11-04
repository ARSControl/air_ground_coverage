"""Replay uses only past recorded packets and never reruns estimation."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from docker.replay_sitl import MissionReplay, frame_times
from src.core.deployment_scenario import build_scenario


def fixture():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    identity = build_scenario(manifest).scenario_hash
    report = dict(scenario_hash=identity, vehicles={"uav_1": {"role": "ground"}})
    points = [[0, 0], [1, 0], [0, 1], [1, 1]]

    def target(t, value, version):
        return dict(
            topic="aerial_target",
            received_at=t,
            data=dict(
                scenario_hash=identity,
                shape=[2, 2],
                query_points=points,
                high_mean=[value] * 4,
                high_variance=[0.2] * 4,
                version=version,
            ),
        )

    def pose(t, position):
        return dict(
            topic="pose",
            received_at=t,
            data=dict(scenario_hash=identity, vehicle_id="uav_1", position=position),
        )

    events = [
        pose(10.0, [0, 0]),
        target(11.0, 1.0, 1),
        pose(12.0, [1, 0]),
        target(13.0, 2.0, 2),
    ]
    return report, events, manifest


def test_latest_posterior_and_progressive_path_without_future_information():
    report, events, manifest = fixture()
    replay = MissionReplay(report, events, manifest)
    assert replay.snapshot(0.0)[0] is None
    target, paths = replay.snapshot(1.5)
    assert target["version"] == 1
    np.testing.assert_array_equal(paths["uav_1"], [[0, 0]])
    target, paths = replay.snapshot(3.0)
    assert target["version"] == 2
    np.testing.assert_array_equal(paths["uav_1"], [[0, 0], [1, 0]])
    # Seeking backward must restore earlier state.
    assert replay.snapshot(1.0)[0]["version"] == 1


def test_reject_mismatched_scenario_or_recorded_target():
    report, events, manifest = fixture()
    report["scenario_hash"] = "wrong"
    with pytest.raises(ValueError, match="scenario"):
        MissionReplay(report, events, manifest)
    report, events, manifest = fixture()
    events[1]["data"]["scenario_hash"] = "wrong"
    with pytest.raises(ValueError, match="scenario"):
        MissionReplay(report, events, manifest)


def test_frame_clock_speed_and_final_frame():
    np.testing.assert_allclose(frame_times(1.0, fps=4, speed=2), [0, 0.5, 1])
    assert frame_times(1.1, fps=4, speed=2)[-1] == 1.1
    for fps, speed in [(0, 1), (10, 0), (10, float("nan"))]:
        with pytest.raises(ValueError):
            frame_times(1.0, fps=fps, speed=speed)


def test_empty_recording_is_rejected():
    report, _, manifest = fixture()
    with pytest.raises(ValueError, match="posterior"):
        MissionReplay(report, [], manifest)


def test_exports_playable_video_and_preview_without_mutating_inputs(tmp_path):
    import json
    from matplotlib import animation
    from docker.replay_sitl import render

    if not animation.writers.is_available("ffmpeg"):
        pytest.skip("ffmpeg unavailable")
    report, events, manifest = fixture()
    original = json.dumps([report, events, manifest], sort_keys=True)
    output = tmp_path / "replay.mp4"
    result = render(
        MissionReplay(report, events, manifest), output, fps=2, speed=3, dpi=40
    )
    assert result["frames"] == 3
    assert result["posterior_packets"] == 2
    assert output.stat().st_size > 0
    assert output.with_suffix(".png").is_file()
    assert json.loads(output.with_suffix(".replay.json").read_text()) == result
    assert json.dumps([report, events, manifest], sort_keys=True) == original


def test_old_report_with_overwritten_event_log_is_rejected():
    report, events, manifest = fixture()
    report["timestamp"] = 9.0
    with pytest.raises(ValueError, match="completion"):
        MissionReplay(report, events, manifest)
