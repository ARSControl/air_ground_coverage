"""Exact 2D velocity projection onto circular CBF halfplanes and a speed disk.

Assumes planar velocity tracking; no physical acceleration/tracking guarantee.
"""
import itertools
import numpy as np


def filter_velocity(position, nominal, centers, radii, alpha, max_speed, *,
                    bounds=None, horizon=2.5):
    """Return closest feasible velocity, or None for invalid/unsafe inputs.

    Radii include the caller's vehicle/tracking clearance. Geofence constraints
    require the held velocity's horizon endpoint to remain within bounds.
    """
    try:
        p, u = np.asarray(position, float), np.asarray(nominal, float)
        c = np.asarray(centers, float).reshape((-1, 2))
        r = np.asarray(radii, float)
        if (p.shape != (2,) or u.shape != (2,) or r.shape != (len(c),)
                or not all(np.all(np.isfinite(x)) for x in (p, u, c, r, [alpha, max_speed, horizon]))
                or np.any(r <= 0) or min(alpha, max_speed, horizon) <= 0):
            return None
        d = p - c
        h = np.sum(d*d, axis=1) - r*r
        if np.any(h < 0):
            return None
        # A v <= b, equivalent to 2 (p-c) . v >= -alpha h.
        a, b = list(-2*d), list(alpha*h)
        if bounds is not None:
            box = np.asarray(bounds, float)
            if box.shape != (2, 2) or not np.all(np.isfinite(box)) or np.any(box[:, 0] > p) or np.any(p > box[:, 1]):
                return None
            for k in range(2):
                axis = np.eye(2)[k]
                a.extend([axis, -axis])
                b.extend([(box[k, 1]-p[k])/horizon, (p[k]-box[k, 0])/horizon])
        a, b = np.asarray(a).reshape((-1, 2)), np.asarray(b)
        candidates = [np.zeros(2), u * min(1., max_speed / max(np.linalg.norm(u), 1e-30))]
        for normal, rhs in zip(a, b):
            n2 = normal @ normal
            if n2 <= 1e-24:
                continue
            candidates.append(u - normal * ((normal @ u - rhs)/n2))
            foot = normal * rhs/n2
            remainder = max_speed**2 - foot @ foot
            if remainder >= 0:
                tangent = np.array([-normal[1], normal[0]]) / np.sqrt(n2)
                candidates.extend([foot + np.sqrt(remainder)*tangent, foot - np.sqrt(remainder)*tangent])
        for i, j in itertools.combinations(range(len(a)), 2):
            matrix = a[[i, j]]
            if abs(np.linalg.det(matrix)) > 1e-12:
                candidates.append(np.linalg.solve(matrix, b[[i, j]]))
        feasible = [v for v in candidates if np.linalg.norm(v) <= max_speed + 1e-10 and np.all(a @ v <= b + 1e-10)]
        return min(feasible, key=lambda v: float(np.sum((v-u)**2))) if feasible else None
    except (ValueError, TypeError, OverflowError):
        return None
