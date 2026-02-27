import numpy as np
import os
import sys
import ot
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.utils.voronoi import compute_voronoi_partitioning

def eval_effectiveness(states, grid_points, target_density, fov_degrees, robot_range):
    """
    Evaluate the effectiveness of each robot by summing target_density values
    for points inside the voronoi region of robot i and within its angular FOV.

    Args:
        states: array of shape (n_robots, 3) with [x, y, theta]
        grid_points: array of shape (N, 2) with grid point coordinates
        target_density: array of shape (N,) or (H, W) with density values
        fov_degrees: field of view angle in degrees
        robot_range: maximum sensing range

    Returns:
        Array of effectiveness values for each robot
    """
    if target_density.ndim == 2:
        target_density = target_density.ravel()

    voronoi_masks = compute_voronoi_partitioning(
        grid_points, states[:, :2], robot_range
    )

    fov_rad = np.deg2rad(fov_degrees)
    half_fov = fov_rad / 2

    effectiveness = 0.0

    for i, state in enumerate(states):
        x = state[0]
        y = state[1]
        theta = state[2]
        voronoi_indices = np.where(voronoi_masks[i])[0]

        if len(voronoi_indices) == 0:
            continue

        voronoi_points = grid_points[voronoi_indices]
        dx = voronoi_points[:, 0] - x
        dy = voronoi_points[:, 1] - y
        distances = np.sqrt(dx * dx + dy * dy)

        angles = np.arctan2(dy, dx)
        angle_diff = angles - theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        fov_mask = (distances <= robot_range) & (np.abs(angle_diff) <= half_fov)

        fov_indices = voronoi_indices[fov_mask]
        effectiveness += np.sum(target_density[fov_indices])
    
    total_density = np.sum(target_density)

    return effectiveness #/ total_density

def eval_norm_effectiveness(states, grid_points, target_density, fov_degrees, robot_range, env_area):
    """
    Evaluate the normalized effectiveness of each robot by summing target_density values
    for points inside the voronoi region of robot i and within its angular FOV, then normalizing by maximum sensible density.

    Args:
        states: array of shape (n_robots, 3) with [x, y, theta]
        grid_points: array of shape (N, 2) with grid point coordinates
        target_density: array of shape (N,) or (H, W) with density values
        fov_degrees: field of view angle in degrees
        robot_range: maximum sensing range
    Returns:
        Normalized effectiveness value (between 0 and 1)
    """
    n = states.shape[0]
    n_pts = grid_points.shape[0]
    sens_area = np.deg2rad(fov_degrees) * robot_range**2 / 2
    Amax = sens_area * n
    gamma = Amax / env_area

    # stack target_density if 2D
    if target_density.ndim == 2:
        target_density = target_density.ravel()

    # Sort grid points by density
    sorted_indices = np.argsort(target_density)[::-1]
    sorted_density = target_density[sorted_indices]
    top_density = sorted_density[:int(gamma * n_pts)]
    phi_max = np.sum(top_density)
    phi = eval_effectiveness(states, grid_points, target_density, fov_degrees, robot_range)
    norm_effectiveness = phi / (phi_max + 1e-10)
    return norm_effectiveness



def eval_kl_divergence(target_density, sensed_density):
    """
    Evaluate KL divergence between target density and sensed density.

    Args:
        target_density: array of shape (N,) or (H, W) with target density values
        sensed_density: array of shape (N,) or (H, W) with sensed density values
    Returns:
        KL divergence value
    """
    if target_density.ndim == 2:
        target_density = target_density.ravel()
    if sensed_density.ndim == 2:
        sensed_density = sensed_density.ravel()

    eps = 1e-10
    p = target_density / (np.sum(target_density) + eps)
    q = sensed_density / (np.sum(sensed_density) + eps)
    q = np.clip(q, 0, None)
    kl = np.sum(p * np.log((p + eps) / (q + eps)))
    return kl

def eval_wasserstein(target_density, sensed_density, eps=1e-12):
    """
    Evaluate Wasserstein distance between target density and sensed density.

    Args:
        target_density: array of shape (N,) or (H, W) with target density values
        sensed_density: array of shape (N,) or (H, W) with sensed density values
    Returns:
        Wasserstein distance value
    """
    h, w = int(np.sqrt(target_density.shape)), int(np.sqrt(target_density.shape))
    p = target_density.ravel().astype(np.float64)
    q = sensed_density.ravel().astype(np.float64)

    p = np.maximum(p, 0)
    q = np.maximum(q, 0)
    p /= (np.sum(p))
    q /= (np.sum(q))
    q *= np.sum(p)

    x, y = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    coords = np.stack([x.ravel(), y.ravel()], axis=1)

    M = ot.dist(coords, coords)
    w_dist = ot.emd2(p, q, M)
    return w_dist