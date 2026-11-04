"""Monotonic fixed-duration mission and stop behavior without ROS."""

import pytest

from docker.timed_mission import MissionClock


def test_deadline_starts_with_first_start_request_and_holds_all():
    mission = MissionClock(["a", "b", "c"], duration=10, service_timeout=3)
    assert mission.begin(100) == "start"
    mission.start_results.update(a=True, b=True, c=True)
    assert mission.tick(100.5) is None
    assert mission.state == "running"
    assert mission.tick(109.99) is None
    assert mission.tick(110) == "hold"
    assert mission.reason == "duration_elapsed"
    assert mission.tick(110.1) is None
    mission.hold_results.update(a=True, b=True, c=True)
    assert mission.tick(110.2) == "finished"
    assert mission.success


def test_partial_start_failure_and_cancel_hold_entire_team():
    mission = MissionClock(["a", "b"], duration=10, service_timeout=3)
    mission.begin(0)
    mission.start_results["a"] = True
    mission.start_results["b"] = False
    assert mission.tick(0.2) == "hold"
    assert mission.reason == "start_rejected"
    assert not mission.success
    mission.hold_results.update(a=True, b=True)
    assert mission.tick(0.3) == "finished"
    assert not mission.success
    mission = MissionClock(["a"], duration=10, service_timeout=3)
    mission.begin(0)
    assert mission.tick(0.1, cancel=True) == "hold"
    assert mission.reason == "cancelled"


def test_start_timeout_and_hold_timeout_do_not_report_success():
    mission = MissionClock(["a"], duration=10, service_timeout=2)
    mission.begin(0)
    assert mission.tick(2) == "hold"
    assert mission.reason == "start_timeout"
    assert mission.tick(4) == "finished"
    assert mission.state == "failed"


def test_short_duration_and_unhealthy_runtime_stop_promptly():
    mission = MissionClock(["a"], duration=0.5, service_timeout=3)
    mission.begin(0)
    assert mission.tick(0.5) == "hold"
    mission = MissionClock(["a"], duration=10, service_timeout=3)
    mission.begin(0)
    mission.start_results["a"] = True
    mission.tick(0.1)
    assert mission.tick(0.2, healthy=False) == "hold"
    assert mission.reason == "pipeline_unhealthy"


@pytest.mark.parametrize("duration", [0, -1, float("nan"), float("inf")])
def test_invalid_duration_rejected(duration):
    with pytest.raises(ValueError):
        MissionClock(["a"], duration=duration, service_timeout=2)


def test_unhealthy_snapshot_preserves_reasons_and_distinguishes_target_age():
    from docker.timed_mission import health_failure_snapshot

    report = dict(
        timestamp=10.0,
        reasons=["uav_1: no observed fresh ROS command"],
        central=dict(timestamp=9.5, posterior_version=8),
        command_stats={"uav_1": dict(last_received_at=9.7)},
        vehicles={
            "uav_1": dict(
                agent=dict(state="MISSION", last_command_time=9.7),
                pose=dict(timestamp=9.9),
                plan=dict(timestamp=9.0),
            )
        },
    )
    target = dict(
        timestamp=7.5, posterior_timestamp=7.0, version=8, update_duration_s=1.45
    )
    snapshot = health_failure_snapshot(report, target)
    assert snapshot["reasons"] == report["reasons"]
    assert snapshot["target_age_s"] == 2.5
    assert snapshot["central_status_age_s"] == 0.5
    assert snapshot["vehicles"]["uav_1"]["observed_command_age_s"] == pytest.approx(0.3)
    assert snapshot["target"]["version"] == 8
    report["reasons"].clear()
    assert snapshot["reasons"]
