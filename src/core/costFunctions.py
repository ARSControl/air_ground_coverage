import numpy as np
import casadi as ca

import numba
from numba import njit, prange

#### 
# Cost function for Discrete PDF MPC
####
# def control_effort_cost(u, R):
#   """
#   u: control input (nu, T)
#   R: weight matrix
#   """
#   costs = ca.vertcat(*[ca.mtimes([U[:,i].T, R, U[:,i]]) for i in range(u.shape[1])])
#   cost = ca.sum1(costs)
#   return cost

def collision_cost(x, x_obs, Ds, alpha=10.0, beta=5.0):
  """
  x: state [x, y]
  x_obs: obstacle state [x, y]
  Ds: safety distance
  """
  # dist = ca.sqrt(ca.sum1(x - x_obs)**2)
  # cost = 1 / (dist - Ds)
  dist_sq = ca.sumsqr(x - x_obs)
  cost = alpha * ca.exp(-beta * (dist_sq - Ds**2))

  return cost

def coverage_cost(
    robot_pos,          # SX (1, 2)
    grid_points,        # DX (GRID_CELLS**2, 2)
    weights             # np.ndarray (GRID_CELLS**2,) 
):
  """
  Compute coverage cost for a robot at position `robot_pos` given a set of `grid_points` and their corresponding `weights`.
  
  :param robot_pos: 2D position of the robot
  :param grid_points: Grid points of the environment
  :param weights: pdf[k] if k inside robot's Voronoi region, else 0
  """
  diff = grid_points - ca.repmat(robot_pos.T, grid_points.shape[0], 1)
  d2 = ca.sum2(diff**2)
  return ca.dot(d2, weights)

def limfov_coverage_cost(state, grid_points, weights, r_max=5.0, half_fov=np.pi, k=10.0):
    """
    Optimized FOV coverage cost with smooth mask for better IPOPT convergence.
    
    :param state: [x, y, theta] robot state
    :param grid_points: Grid points (N, 2) - should be ca.DM for efficiency
    :param weights: Importance weights (N,)  
    :param half_fov: Half of FOV angle (radians), pre-computed as alpha/2
    :param k: Steepness of sigmoid for smooth mask
    """
    pos = state[:2]
    theta = state[2]
    
    diff = grid_points - ca.repmat(pos.T, grid_points.shape[0], 1) 
    r2 = ca.sum2(diff**2)
    # f = ca.exp(-r2 / (r_max**2))  # Gaussian decay with distance
    f = 2 - r2 / r_max**2  # Quadratic decay, faster to compute than exp
    # atan2 returns [-pi, pi], no normalization needed for FOV < 180°
    angles = ca.atan2(diff[:,1], diff[:,0]) - theta
    
    # Smooth sigmoid mask: 1 when |angle| < half_fov, 0 otherwise
    # This gives better gradients for IPOPT than hard threshold
    mask = 1.0 / (1.0 + ca.exp(k * (ca.fabs(angles) - half_fov)))
    
    return -ca.sum1(f * mask * weights)

def orientation_cost(state, grid_points, weights, r_max):
    """
    Orientation-only cost: align heading with centroid of sensed mass

    theta: scalar (heading)
    pos:   (2,) robot position
    grid_points: (N,2) ca.DM or SX
    weights: (N,) ca.DM or SX (Voronoi + FOV filtered)
    """
    pos = state[:2]
    theta = state[2]
    diff = grid_points - ca.repmat(pos.T, grid_points.shape[0], 1)
    r2 = ca.sum2(diff**2)

    # Radial sensing function
    f = ca.exp(-r2 / r_max**2)

    w = f * weights
    mass = ca.sum1(w) + 1e-6  # avoid division by zero

    centroid = ca.sum2(ca.reshape(w, (w.shape[0], 1)) * grid_points) / mass

    desired_angle = ca.atan2(
        centroid[1] - pos[1],
        centroid[0] - pos[0]
    )

    # Smooth, bounded orientation error
    return 1 - ca.cos(theta - desired_angle)
