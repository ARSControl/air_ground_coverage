"""Central compute cannot consume the next posterior's freshness on arrival."""

from pathlib import Path

import pytest
import yaml

from docker.deployment_publication import CentralUpdateLoop
from docker.deployment_runtime import DeploymentRuntime


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def runtime():
    manifest = yaml.safe_load(
        Path("docker/scenarios/sitl_three_drone.yaml").read_text()
    )
    return DeploymentRuntime(manifest, grid_points=5)


def test_observed_slow_update_keeps_publication_gap_below_two_seconds():
    model = runtime()
    clock = Clock()
    emitted = []
    statuses = []
    model.sample(
        dict(
            scenario_hash=model.scenario.scenario_hash,
            vehicle_id="uav_3",
            request_id="initial",
            timestamp=0.0,
            position=[15.0, 15.0],
            heading=0.0,
        )
    )
    loop = CentralUpdateLoop(
        model,
        clock=clock,
        wall_clock=clock,
        publish_target=lambda p: emitted.append((clock(), p)),
        publish_status=lambda: statuses.append(clock()),
    )
    loop.tick()
    assert emitted[-1][1]["model_updated"]
    version = emitted[-1][1]["version"]
    # Repeat the measured 1.45 s compute time after a 1 s idle interval.
    original = model.tick

    def delayed(timestamp):
        clock.now += 1.45
        return original(timestamp)

    model.tick = delayed
    clock.now = 1.0
    loop.tick()
    before, after = emitted[-2:]
    assert before[0] == 1.0
    assert before[1]["update_in_progress"] is True
    assert after[0] == pytest.approx(2.45)
    assert after[1]["timestamp"] == pytest.approx(2.45)
    assert after[1]["posterior_timestamp"] == 0.0
    assert after[1]["version"] == version
    assert after[1]["model_updated"] is False
    assert max(b[0] - a[0] for a, b in zip(emitted, emitted[1:])) < 2.0
    # The next attempt starts one full period after completion, without catch-up.
    attempts = len(emitted)
    clock.now = 2.5
    loop.tick()
    assert len(emitted) == attempts
    clock.now = 3.45
    loop.tick()
    assert len(emitted) == attempts + 2


def test_failure_does_not_continuously_refresh_the_old_posterior():
    model = runtime()
    clock = Clock()
    emitted = []
    model.sample(
        dict(
            scenario_hash=model.scenario.scenario_hash,
            vehicle_id="uav_3",
            request_id="initial",
            timestamp=0.0,
            position=[15.0, 15.0],
            heading=0.0,
        )
    )
    loop = CentralUpdateLoop(
        model,
        clock=clock,
        wall_clock=clock,
        publish_target=lambda p: emitted.append((clock(), p)),
        publish_status=lambda: None,
    )
    loop.tick()
    snapshot = model.estimator.estimator.latest_posterior

    def failed(_):
        clock.now += 0.2
        raise RuntimeError("injected update failure")

    model.tick = failed
    clock.now = 1.0
    with pytest.raises(RuntimeError):
        loop.tick()
    count = len(emitted)  # A pre-update heartbeat is permitted before failure is known.
    clock.now = 3.0
    with pytest.raises(RuntimeError):
        loop.tick()
    assert len(emitted) == count
    assert model.estimator.estimator.latest_posterior is snapshot


def test_image_pins_accelerated_single_thread_numerical_stack():
    dockerfile = Path("docker/Dockerfile").read_text()
    assert "--only-binary=:all:" in dockerfile
    assert "--ignore-installed" in dockerfile
    assert "numpy==1.26.4" in dockerfile and "scipy==1.13.1" in dockerfile
    assert "OPENBLAS_NUM_THREADS=1" in dockerfile


def test_full_240_second_virtual_mission_has_no_command_dropouts():
    from docker.deployment_timing_validation import simulate_timing

    result = simulate_timing(duration=240.0, compute_time=1.45)
    metrics = result["metrics"]
    assert metrics["fixed_command_rejections"] == 0
    assert metrics["fixed_max_target_age_s"] < 2.0
    assert metrics["old_max_target_age_s"] > 2.0
    assert metrics["old_command_rejections"] > 0
    assert metrics["fixed_command_checks"] >= 14400


def test_successful_retry_resumes_publication_after_failure():
    model = runtime()
    clock = Clock()
    emitted = []
    model.sample(
        dict(
            scenario_hash=model.scenario.scenario_hash,
            vehicle_id="uav_3",
            request_id="initial",
            timestamp=0.0,
            position=[15.0, 15.0],
            heading=0.0,
        )
    )
    loop = CentralUpdateLoop(
        model,
        clock=clock,
        wall_clock=clock,
        publish_target=emitted.append,
        publish_status=lambda: None,
    )
    loop.tick()
    original = model.tick

    def failed(_):
        raise RuntimeError("failed attempt")

    model.tick = failed
    clock.now = 1.0
    with pytest.raises(RuntimeError):
        loop.tick()
    assert loop.failed
    count = len(emitted)
    model.tick = original
    model.sample(
        dict(
            scenario_hash=model.scenario.scenario_hash,
            vehicle_id="uav_1",
            request_id="retry",
            timestamp=2.0,
            position=[15.0, 15.0],
            heading=0.0,
        )
    )
    clock.now = 3.0
    loop.tick()
    assert len(emitted) == count + 1  # No pre-update heartbeat after a known failure.
    assert emitted[-1]["version"] == 2 and emitted[-1]["model_updated"]
    assert loop.failed is False
