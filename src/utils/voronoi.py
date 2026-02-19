import numpy as np
import numba
from numba import njit, prange


def compute_voronoi_region(xy_grid, robot_positions, robot_idx, robot_range, grid_spacing):
  """
  Compute which grid points belong to robot i's Voronoi region.
  
  Args:
      robot_positions: array of shape (n_robots, 2)
      robot_idx: index of the robot
  
  Returns:
      Boolean mask indicating points in Voronoi region
  """
  # Compute distances from all grid points to all robots
  dists = np.linalg.norm(
      xy_grid[:, np.newaxis, :] - robot_positions[np.newaxis, :, :],
      axis=2
  )
  
  # Points belong to robot i if it's the closest
  closest_robot = np.argmin(dists, axis=1)
  voronoi_mask = (closest_robot == robot_idx)

  # limit to range
  robot = robot_positions[robot_idx]
  dists_to_robot = np.linalg.norm(xy_grid - robot, axis=1)
  voronoi_mask = np.logical_and(voronoi_mask, dists_to_robot <= robot_range * grid_spacing)
  
  return voronoi_mask

@numba.njit(parallel=True, cache=True)
def compute_voronoi_partitioning(xy_grid, robot_positions, robot_range):
    n_points = xy_grid.shape[0]
    n_robots = robot_positions.shape[0]
    
    # Compute squared distances manually (faster, Numba-compatible)
    dists_sq = np.empty((n_points, n_robots))
    for i in numba.prange(n_points):
        for j in range(n_robots):
            dx = xy_grid[i, 0] - robot_positions[j, 0]
            dy = xy_grid[i, 1] - robot_positions[j, 1]
            dists_sq[i, j] = dx * dx + dy * dy
    
    closest = np.argmin(dists_sq, axis=1)
    
    masks = []
    range_sq = robot_range * robot_range
    for i in range(n_robots):
        mask = (closest == i)
        mask &= (dists_sq[:, i] <= range_sq)
        masks.append(mask)
    
    return masks

@numba.njit(parallel=True, cache=True)
def compute_anisotropic_voronoi_partitioning(xy_grid, robot_states, robot_range, gamma=5.0):
    """
    Anisotropic Voronoi partitioning aligned with robot heading.

    Args:
        xy_grid: (N,2)
        robot_states: (n_robots, 3) -> [x, y, theta]
        robot_range: sensing radius
        gamma: anisotropy factor (>1 penalizes lateral direction)

    Returns:
        list of boolean masks (same format as your current function)
    """
    n_points = xy_grid.shape[0]
    n_robots = robot_states.shape[0]

    dists = np.empty((n_points, n_robots))
    range_sq = robot_range * robot_range

    for j in range(n_robots):

        px = robot_states[j, 0]
        py = robot_states[j, 1]
        theta = robot_states[j, 2]

        c = np.cos(theta)
        s = np.sin(theta)

        # Q = R diag(1,gamma) R^T
        q11 = c*c + gamma*s*s
        q12 = (1-gamma)*c*s
        q21 = q12
        q22 = s*s + gamma*c*c

        for i in numba.prange(n_points):

            dx = xy_grid[i, 0] - px
            dy = xy_grid[i, 1] - py

            # Quadratic form (x-p)^T Q (x-p)
            dists[i, j] = dx*(q11*dx + q12*dy) + dy*(q21*dx + q22*dy)

    closest = np.argmin(dists, axis=1)

    masks = []
    for j in range(n_robots):
        mask = (closest == j)

        # still enforce circular sensing range
        dx = xy_grid[:, 0] - robot_states[j, 0]
        dy = xy_grid[:, 1] - robot_states[j, 1]
        mask &= (dx*dx + dy*dy <= range_sq)

        masks.append(mask)

    return masks


def agent_fov(state, robot_range, fov_angle, num_points=10):
    """
    Compute the field of view (FOV) polygon for an agent given its state.
    
    Args:
        state: (3,) array [x, y, theta]
        range: maximum sensing range
        fov_angle: field of view angle in radians
    Returns:
        fov_polygon: (3, 2) array of vertices of the FOV triangle
    """
    x, y, theta = state
    th_arc = np.linspace(-fov_angle/2, fov_angle/2, num=num_points) + theta
    arc_points = np.column_stack([x + robot_range * np.cos(th_arc),
                                  y + robot_range * np.sin(th_arc)])
    fov_polygon = np.vstack([[x, y], arc_points, [x, y]])
    return fov_polygon