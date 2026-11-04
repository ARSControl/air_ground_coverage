"""Sequential deployment adapter for one central multi-fidelity estimator.

This module does not implement ROS transport, clock synchronization, source
authentication, motion control, or obstacle-aware density masking.
"""

import math

import numpy as np

from .epistemic_sample_filter import EpistemicFilterSettings
from .density import build_aerial_target
from .multifidelity_estimator import (
    CentralAsynchronousEstimator,
    EstimatorSettings,
    UpdateStatus,
)
from .multifidelity_gp import MultiFidelityGaussianProcess, RBFKernel
from .observations import Fidelity, Observation, RetentionConfig


class DeploymentEstimator:
    """Accept idempotent batches and explicitly update on a fixed period."""

    def __init__(self, manifest, scenario_hash, *, grid_points=21, max_batches=10000):
        if not isinstance(scenario_hash, str) or not scenario_hash:
            raise ValueError("scenario hash is required")
        if (
            isinstance(grid_points, bool)
            or not isinstance(grid_points, int)
            or grid_points < 2
            or grid_points > 201
        ):
            raise ValueError("grid_points must be between 2 and 201")
        if (
            isinstance(max_batches, bool)
            or not isinstance(max_batches, int)
            or max_batches < 1
        ):
            raise ValueError("invalid batch limit")
        settings = manifest["estimator"]
        grid = manifest["map"]
        origin = np.asarray(grid["origin_m"], dtype=float)
        resolution = float(grid["resolution_m"])
        rows, columns = grid["shape_rows_columns"]
        if (
            origin.shape != (2,)
            or not np.all(np.isfinite(origin))
            or not math.isfinite(resolution)
            or resolution <= 0
            or rows < 2
            or columns < 2
        ):
            raise ValueError("invalid grid")
        self.period = float(settings["update_period_s"])
        if not math.isfinite(self.period) or self.period <= 0:
            raise ValueError("invalid update period")
        xs = np.linspace(origin[0], origin[0] + (columns - 1) * resolution, grid_points)
        ys = np.linspace(origin[1], origin[1] + (rows - 1) * resolution, grid_points)
        xx, yy = np.meshgrid(xs, ys)
        wx = np.full(grid_points, (xs[-1] - xs[0]) / (grid_points - 1))
        wy = np.full(grid_points, (ys[-1] - ys[0]) / (grid_points - 1))
        wx[[0, -1]] *= 0.5
        wy[[0, -1]] *= 0.5
        low, delta = settings["low_kernel"], settings["discrepancy_kernel"]
        gp = MultiFidelityGaussianProcess(
            rho=settings["rho"],
            low_kernel=RBFKernel(low["length_scale_m"], low["variance"]),
            discrepancy_kernel=RBFKernel(delta["length_scale_m"], delta["variance"]),
            jitter=settings["jitter"],
        )
        retention = settings["retention"]
        policy = retention.get("policy", "newest")
        if policy not in {"newest", "paper_epistemic"}:
            raise ValueError("unknown retention policy")
        sample_filter = EpistemicFilterSettings(
            enabled=policy == "paper_epistemic",
            relative_error=retention.get("relative_error", 0.05),
            confidence_z=retention.get("confidence_z", 1.96),
            criterion=retention.get("criterion", "native_epistemic"),
            on_full=retention.get("on_full", "reject"),
            replacement_min_improvement=retention.get(
                "replacement_min_improvement", 1e-8
            ),
        )
        total = retention.get("max_total_samples")
        if total is not None and (
            "max_low_samples" in retention or "max_high_samples" in retention
        ):
            raise ValueError(
                "choose a shared budget or separate fidelity budgets, not both"
            )
        self.estimator = CentralAsynchronousEstimator(
            EstimatorSettings(
                gp=gp,
                sample_filter=sample_filter,
                total_retention=total,
                low_retention=RetentionConfig(
                    total if total is not None else retention["max_low_samples"], 0
                ),
                high_retention=RetentionConfig(
                    total if total is not None else retention["max_high_samples"], 0
                ),
                query_points=np.column_stack((xx.ravel(), yy.ravel())),
                query_shape=xx.shape,
                integration_weights=np.outer(wy, wx).ravel(),
            )
        )
        self._roles = {}
        for vehicle in manifest["vehicles"]:
            vid, role = vehicle["id"], vehicle["role"]
            if (
                not isinstance(vid, str)
                or not vid
                or vid in self._roles
                or role not in {"ground", "aerial"}
            ):
                raise ValueError("invalid vehicle roles")
            self._roles[vid] = Fidelity.HIGH if role == "ground" else Fidelity.LOW
        self._scenario_hash = scenario_hash
        self._max_batches = max_batches
        self._batches = {}
        self._next_update = None
        self._last_time = None
        self._latest_pending_time = None

    def submit_batch(self, scenario_hash, vehicle_id, request_id, observations):
        """Queue a batch exactly once within the current process session."""
        if scenario_hash != self._scenario_hash or vehicle_id not in self._roles:
            raise ValueError("scenario or vehicle mismatch")
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("invalid request ID")
        batch = tuple(observations)
        if not all(
            isinstance(o, Observation)
            and o.robot_id == vehicle_id
            and o.fidelity is self._roles[vehicle_id]
            for o in batch
        ):
            raise ValueError("invalid observation batch")
        key = (vehicle_id, request_id)
        if key in self._batches:
            if self._batches[key] != batch:
                raise ValueError("conflicting batch")
            return 0
        if len(self._batches) >= self._max_batches:
            raise ValueError("batch cache full")
        count = self.estimator.submit_many(batch)
        if batch:
            newest = max(o.timestamp for o in batch)
            self._latest_pending_time = (
                newest
                if self._latest_pending_time is None
                else max(newest, self._latest_pending_time)
            )
        self._batches[key] = batch
        return count

    def aerial_target(self, lambda_interest=1.0, lambda_uncertainty=0.25):
        """Return the existing normalized HIGH posterior interest/uncertainty target."""
        snapshot = self.estimator.latest_posterior
        if snapshot is None:
            return None
        return build_aerial_target(
            snapshot.density,
            snapshot.high_variance,
            lambda_interest,
            lambda_uncertainty,
            snapshot.integration_weights,
        )

    def update_due(self, timestamp):
        """Read-only scheduler check used before publishing a pre-update heartbeat."""
        now = float(timestamp)
        if not math.isfinite(now) or (
            self._last_time is not None and now < self._last_time
        ):
            raise ValueError("update time must be finite and nondecreasing")
        return self._next_update is None or now >= self._next_update

    def complete_update(self, timestamp):
        """Keep a full update period after actual compute completion in deployment."""
        now = float(timestamp)
        if not math.isfinite(now) or (
            self._last_time is not None and now < self._last_time
        ):
            raise ValueError("completion time must be finite and nondecreasing")
        self._last_time = now
        self._next_update = now + self.period

    def update(self, timestamp):
        """Attempt at most one update per period; return None when not due."""
        now = float(timestamp)
        if not math.isfinite(now) or (
            self._last_time is not None and now < self._last_time
        ):
            raise ValueError("update time must be finite and nondecreasing")
        self._last_time = now
        if self._next_update is not None and now < self._next_update:
            return None
        if self._latest_pending_time is not None and self._latest_pending_time > now:
            raise ValueError("pending observation is in the future")
        self._next_update = now + self.period
        report = self.estimator.update(now)
        if report.status in {UpdateStatus.UPDATED, UpdateStatus.NO_NEW_DATA}:
            self._latest_pending_time = None
        return report
