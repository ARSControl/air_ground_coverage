"""Manifest adapter for the optional ground command CBF."""
import math
from src.core.planar_cbf import filter_velocity


class ObstacleFilter:
    def __init__(self, manifest, role):
        config = manifest['control'].get('obstacle_cbf', {})
        self.enabled = config.get('enabled', False)
        if not isinstance(self.enabled, bool):
            raise ValueError('CBF enabled must be boolean')
        self.enabled = self.enabled and role == 'ground'
        self.last_status = 'disabled'
        if not self.enabled:
            return
        self.alpha = float(config['alpha_per_s'])
        clearance = float(config['clearance_m'])
        if not math.isfinite(self.alpha) or self.alpha <= 0 or not math.isfinite(clearance) or clearance < 0:
            raise ValueError('invalid CBF gain or clearance')
        geometry = manifest['map']['obstacles']
        if geometry['kind'] != 'circles':
            raise ValueError('CBF requires circular obstacles')
        self.centers, self.radii = [], []
        for circle in geometry['circles']:
            center, radius = circle['center_m'], float(circle['radius_m'])
            if len(center) != 2 or not all(math.isfinite(x) for x in center) or not math.isfinite(radius) or radius <= 0:
                raise ValueError('invalid CBF obstacle')
            self.centers.append(center)
            self.radii.append(radius + clearance)
        self.last_status = "awaiting_command"

    def apply(self, position, velocity, max_speed, bounds, horizon):
        if not self.enabled:
            return velocity
        result = filter_velocity(position, velocity, self.centers, self.radii,
                                 self.alpha, max_speed, bounds=bounds, horizon=horizon)
        if result is None:
            self.last_status = 'unsafe_or_infeasible_hold'
            return [0.0, 0.0]
        self.last_status = 'filtered' if any(abs(a-b) > 1e-9 for a, b in zip(result, velocity)) else 'nominal'
        return result
