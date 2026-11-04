"""Sequential central sensing and MFGP runtime, independent of ROS."""

from src.core.deployment_estimator import DeploymentEstimator
from src.core.deployment_scenario import build_scenario
from src.core.multifidelity_estimator import UpdateStatus


class DeploymentRuntime:
    """Own one simulated scenario and one central estimator."""

    def __init__(self, manifest, *, grid_points=21):
        self.scenario = build_scenario(manifest)
        self.estimator = DeploymentEstimator(
            manifest, self.scenario.scenario_hash, grid_points=grid_points
        )

    def sample(self, request):
        """Apply an idempotent sample request and queue its observations."""
        required = {
            "scenario_hash",
            "vehicle_id",
            "request_id",
            "timestamp",
            "position",
            "heading",
        }
        if not isinstance(request, dict) or set(request) != required:
            raise ValueError("invalid request fields")
        scenario_hash = request["scenario_hash"]
        vehicle_id = request["vehicle_id"]
        request_id = request["request_id"]
        batch = self.scenario.service.sample(
            scenario_hash,
            vehicle_id,
            request_id,
            request["timestamp"],
            request["position"],
            request["heading"],
        )
        submitted = self.estimator.submit_batch(
            scenario_hash, vehicle_id, request_id, batch
        )
        return {
            "scenario_hash": scenario_hash,
            "vehicle_id": vehicle_id,
            "request_id": request_id,
            "count": len(batch),
            "submitted": submitted,
            "observations": [
                dict(
                    timestamp=o.timestamp,
                    robot_id=o.robot_id,
                    position=list(o.position),
                    value=o.value,
                    fidelity=o.fidelity.value,
                    noise_variance=o.noise_variance,
                )
                for o in batch
            ],
        }

    def tick(self, timestamp):
        """Publish a new target only after a successful scheduled update."""
        report = self.estimator.update(timestamp)
        if report is None:
            return None
        if report.status is UpdateStatus.NO_NEW_DATA and not (
            self.estimator.estimator.settings.sample_filter.enabled
            and self.estimator.estimator.latest_posterior is not None
        ):
            return None
        if report.status is UpdateStatus.FAILED:
            raise RuntimeError(f"estimator update failed: {report.message}")
        snapshot = self.estimator.estimator.latest_posterior
        return {
            "scenario_hash": self.scenario.scenario_hash,
            "version": snapshot.version,
            "timestamp": timestamp,
            "posterior_timestamp": snapshot.timestamp,
            "model_updated": report.status is UpdateStatus.UPDATED,
            "sample_filter": self.estimator.estimator.filter_diagnostics,
            "shape": list(snapshot.query_shape),
            "query_points": snapshot.query_points.tolist(),
            "density": self.estimator.aerial_target().tolist(),
            "high_density": snapshot.density.tolist(),
            "integration_weights": snapshot.integration_weights.tolist(),
            "high_mean": snapshot.high_mean.tolist(),
            "high_variance": snapshot.high_variance.tolist(),
        }
