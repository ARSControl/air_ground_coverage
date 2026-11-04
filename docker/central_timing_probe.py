"""Isolated full-budget numerical timing and slow-ROS-publication regression."""

import argparse
import json
from pathlib import Path
from time import perf_counter, sleep, monotonic

import numpy as np
import scipy
from threadpoolctl import threadpool_info
import yaml

from docker.deployment_runtime import DeploymentRuntime
from docker.deployment_settings import scenario_path
from src.core.observations import Fidelity, Observation
from src.models.sensors import build_fidelity_fields


def benchmark_full_budget(*, require_accelerated=True):
    manifest = yaml.safe_load(Path(scenario_path()).read_text())
    runtime = DeploymentRuntime(manifest)
    scenario = runtime.scenario
    identity = scenario.scenario_hash
    fields = build_fidelity_fields(
        scenario.high_field,
        manifest["field"]["truth_rho"],
        manifest["field"]["low_smoothing_sigma_cells"],
    )
    cells = np.array([(r, c) for r in range(61) for c in range(61)])
    cells = cells[np.any(cells != [25, 25], axis=1)]
    rng = np.random.default_rng(1729)
    low_cells = cells[rng.choice(len(cells), 498, replace=False)]
    outside = cells[np.linalg.norm(cells - [25, 25], axis=1) > 10]
    high_cells = outside[rng.choice(len(outside), 500, replace=False)]
    low = [
        Observation(
            0.0, "uav_3", (15.0, 15.0), float(fields.low[25, 25]), Fidelity.LOW, 0.04
        ),
        Observation(
            0.0, "uav_3", (15.05, 15.0), float(fields.low[25, 25]), Fidelity.LOW, 0.04
        ),
    ]
    low += [
        Observation(
            0.0,
            "uav_3",
            (float(c - 10), float(r - 10)),
            float(fields.low[r, c]),
            Fidelity.LOW,
            0.04,
        )
        for r, c in low_cells
    ]
    high = [
        Observation(
            0.5,
            "uav_1",
            (float(c - 10), float(r - 10)),
            float(fields.high[r, c]),
            Fidelity.HIGH,
            0.0001,
        )
        for r, c in high_cells
    ]
    runtime.estimator.submit_batch(identity, "uav_3", "initial", low)
    runtime.estimator.submit_batch(identity, "uav_1", "initial", high)
    runtime.tick(1.0)
    assert runtime.estimator.estimator.filter_diagnostics["retained_total"] == 1000
    runtime.estimator.submit_batch(
        identity,
        "uav_1",
        "new-high",
        (
            Observation(
                2.0,
                "uav_1",
                (15.0, 15.0),
                float(fields.high[25, 25]),
                Fidelity.HIGH,
                0.0001,
            ),
        ),
    )
    start = perf_counter()
    result = runtime.tick(2.0)
    duration = perf_counter() - start
    assert result["sample_filter"]["replaced"] == 1
    pools = [p for p in threadpool_info() if p["user_api"] == "blas"]
    if require_accelerated:
        assert pools and all(
            p["internal_api"] == "openblas" and p["num_threads"] == 1 for p in pools
        ), pools
        assert duration < 2.0, (
            f"full-budget update exceeds target freshness: {duration}"
        )
    return dict(
        numpy=np.__version__,
        scipy=scipy.__version__,
        blas=pools,
        full_budget_update_s=duration,
        filter=result["sample_filter"],
    )


def slow_ros_publication():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from docker.ros_central import CentralNode

    rclpy.init()
    central = CentralNode(scenario_path())
    observer = Node("slow_publication_probe")
    executor = SingleThreadedExecutor()
    for node in (central, observer):
        executor.add_node(node)
    events = []

    def received(message):
        packet = json.loads(message.data)
        events.append((observer.get_clock().now().nanoseconds * 1e-9, packet))

    subscription = observer.create_subscription(
        String,
        "/mfgp/aerial_target",
        received,
        QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    now = observer.get_clock().now().nanoseconds * 1e-9
    identity = central.runtime.scenario.scenario_hash
    for vehicle, fidelity in [("uav_3", Fidelity.LOW), ("uav_1", Fidelity.HIGH)]:
        central.runtime.estimator.submit_batch(
            identity,
            vehicle,
            "initial",
            (Observation(now, vehicle, (15.0, 15.0), 1.0, fidelity, 0.0001),),
        )

    def spin_until(predicate):
        deadline = monotonic() + 8
        while not predicate() and monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "slow ROS publication probe timeout"

    spin_until(lambda: bool(events))
    original = central.runtime.tick
    pending = {"value": True}

    def delayed(timestamp):
        if pending["value"]:
            pending["value"] = False
            sleep(1.45)  # Deliberately reproduce the measured blocking callback.
        return original(timestamp)

    central.runtime.tick = delayed
    spin_until(
        lambda: any(
            not p["update_in_progress"] and p.get("update_duration_s", 0) > 1.4
            for _, p in events
        )
    )
    slow = [
        (t, p)
        for t, p in events
        if not p["update_in_progress"] and p.get("update_duration_s", 0) > 1.4
    ][-1]
    pre = [
        p
        for _, p in events
        if p["update_in_progress"]
        and p["update_started_at"] == slow[1]["update_started_at"]
    ]
    assert pre, "no cached publication before compute"
    assert slow[0] - slow[1]["timestamp"] < 0.25
    assert 1.4 < slow[1]["timestamp"] - pre[-1]["timestamp"] < 2.0
    assert slow[1]["posterior_timestamp"] < slow[1]["timestamp"]
    result = dict(
        passed=True,
        injected_delay_s=1.45,
        measured_update_s=slow[1]["update_duration_s"],
        completed_packet_age_on_receipt_s=slow[0] - slow[1]["timestamp"],
        before_after_publication_gap_s=slow[1]["timestamp"] - pre[-1]["timestamp"],
    )
    observer.destroy_subscription(subscription)
    for node in (central, observer):
        executor.remove_node(node)
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-only", action="store_true")
    parser.add_argument("--allow-generic-backend", action="store_true")
    args = parser.parse_args()
    result = dict(
        passed=True,
        benchmark=benchmark_full_budget(
            require_accelerated=not args.allow_generic_backend
        ),
    )
    if not args.benchmark_only:
        result["slow_publication"] = slow_ros_publication()
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
