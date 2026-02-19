"""
Motion models for robot dynamics.
Supports single integrator, double integrator, and unicycle models.
Each model has both Numba (JIT-compiled) and CasADi versions for optimization.
"""

import numpy as np
import numba
from numba import njit

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False


# ============================================================================
# Single Integrator Dynamics (point mass, velocity control)
# State: [x, y], Control: [vx, vy]
# ============================================================================

@njit(cache=True)
def single_integrator_numba(state, u, t_step):
    """
    Single integrator dynamics (JIT-compiled).
    
    Args:
        state: (2,) array [x, y]
        u: (2,) array [vx, vy] - velocity control
        t_step: float - time step
    
    Returns:
        new_state: (2,) array after one time step
    """
    new_state = np.empty(2)
    new_state[0] = state[0] + u[0] * t_step
    new_state[1] = state[1] + u[1] * t_step
    return new_state


def single_integrator_casadi(state, u, t_step):
    """
    Single integrator dynamics (CasADi version for optimization).
    
    Args:
        state: CasADi SX/MX [x, y]
        u: CasADi SX/MX [vx, vy]
        t_step: float - time step
    
    Returns:
        new_state: CasADi SX/MX after one time step
    """
    if not CASADI_AVAILABLE:
        raise ImportError("CasADi is required for single_integrator_casadi")
    
    x_new = state[0] + u[0] * t_step
    y_new = state[1] + u[1] * t_step
    return ca.vertcat(x_new, y_new)


# ============================================================================
# Double Integrator Dynamics (point mass with velocity, acceleration control)
# State: [x, y, vx, vy], Control: [ax, ay]
# ============================================================================

@njit(cache=True)
def double_integrator_numba(state, u, t_step):
    """
    Double integrator dynamics (JIT-compiled).
    
    Args:
        state: (4,) array [x, y, vx, vy]
        u: (2,) array [ax, ay] - acceleration control
        t_step: float - time step
    
    Returns:
        new_state: (4,) array after one time step
    """
    new_state = np.empty(4)
    new_state[0] = state[0] + state[2] * t_step  # x + vx * dt
    new_state[1] = state[1] + state[3] * t_step  # y + vy * dt
    new_state[2] = state[2] + u[0] * t_step      # vx + ax * dt
    new_state[3] = state[3] + u[1] * t_step      # vy + ay * dt
    return new_state


def double_integrator_casadi(state, u, t_step):
    """
    Double integrator dynamics (CasADi version for optimization).
    
    Args:
        state: CasADi SX/MX [x, y, vx, vy]
        u: CasADi SX/MX [ax, ay]
        t_step: float - time step
    
    Returns:
        new_state: CasADi SX/MX after one time step
    """
    if not CASADI_AVAILABLE:
        raise ImportError("CasADi is required for double_integrator_casadi")
    
    px = state[0]
    py = state[1]
    vx = state[2]
    vy = state[3]
    ax = u[0]
    ay = u[1]
    
    px_dot = vx
    py_dot = vy
    vx_dot = ax
    vy_dot = ay
    
    x_dot = ca.vertcat(px_dot, py_dot, vx_dot, vy_dot)
    return state + x_dot * t_step


# ============================================================================
# Unicycle Dynamics (differential drive, velocity and steering control)
# State: [x, y, theta], Control: [v, omega]
# ============================================================================

@njit(cache=True)
def unicycle_numba(state, u, t_step):
    """
    Unicycle dynamics (JIT-compiled).
    
    Args:
        state: (3,) array [x, y, theta]
        u: (2,) array [v, omega] - linear and angular velocity
        t_step: float - time step
    
    Returns:
        new_state: (3,) array after one time step
    """
    new_state = np.empty(3)
    new_state[0] = state[0] + u[0] * np.cos(state[2]) * t_step
    new_state[1] = state[1] + u[0] * np.sin(state[2]) * t_step
    new_state[2] = state[2] + u[1] * t_step
    return new_state


def unicycle_casadi(state, u, t_step):
    """
    Unicycle dynamics (CasADi version for optimization).
    
    Args:
        state: CasADi SX/MX [x, y, theta]
        u: CasADi SX/MX [v, omega]
        t_step: float - time step
    
    Returns:
        new_state: CasADi SX/MX after one time step
    """
    if not CASADI_AVAILABLE:
        raise ImportError("CasADi is required for unicycle_casadi")
    
    x = state[0]
    y = state[1]
    theta = state[2]
    v = u[0]
    omega = u[1]
    
    x_new = x + v * ca.cos(theta) * t_step
    y_new = y + v * ca.sin(theta) * t_step
    theta_new = theta + omega * t_step
    
    return ca.vertcat(x_new, y_new, theta_new)


# ============================================================================
# State and Control Dimension Information
# ============================================================================

MODEL_CONFIGS = {
    'single_integrator': {
        'nx': 2,
        'nu': 2,
        'numba_func': single_integrator_numba,
        'casadi_func': single_integrator_casadi
    },
    'double_integrator': {
        'nx': 4,
        'nu': 2,
        'numba_func': double_integrator_numba,
        'casadi_func': double_integrator_casadi
    },
    'unicycle': {
        'nx': 3,
        'nu': 2,
        'numba_func': unicycle_numba,
        'casadi_func': unicycle_casadi
    }
}


def get_dynamics(model_type='double_integrator', backend='numba'):
    """
    Get dynamics function based on model type and backend.
    
    Args:
        model_type: str - one of 'single_integrator', 'double_integrator', 'unicycle'
        backend: str - one of 'numba' (for simulation) or 'casadi' (for optimization)
    
    Returns:
        dynamics function
    
    Example:
        >>> dynamics_sim = get_dynamics('double_integrator', 'numba')
        >>> dynamics_opt = get_dynamics('double_integrator', 'casadi')
    """
    if model_type not in MODEL_CONFIGS:
        raise ValueError(f"Unknown model_type: {model_type}. "
                        f"Available: {list(MODEL_CONFIGS.keys())}")
    
    if backend not in ['numba', 'casadi']:
        raise ValueError(f"Unknown backend: {backend}. Use 'numba' or 'casadi'")
    
    config = MODEL_CONFIGS[model_type]
    
    if backend == 'numba':
        return config['numba_func']
    else:
        return config['casadi_func']


def get_model_config(model_type='double_integrator'):
    """
    Get model configuration (state and control dimensions).
    
    Args:
        model_type: str - one of 'single_integrator', 'double_integrator', 'unicycle'
    
    Returns:
        dict with 'nx' (state dimension) and 'nu' (control dimension)
    """
    if model_type not in MODEL_CONFIGS:
        raise ValueError(f"Unknown model_type: {model_type}. "
                        f"Available: {list(MODEL_CONFIGS.keys())}")
    
    config = MODEL_CONFIGS[model_type]
    return {'nx': config['nx'], 'nu': config['nu']}


@njit(cache=True)
def simulate_trajectory_numba(x_init, u_opt, dt, T, nx, dynamics_func):
    """
    Simulate trajectory using any JIT-compiled dynamics function.
    
    Args:
        x_init: (nx,) initial state
        u_opt: (T, nu) control sequence
        dt: time step
        T: horizon length
        nx: state dimension
        dynamics_func: JIT-compiled dynamics function
    
    Returns:
        planned_traj: (T+1, nx) trajectory
    """
    planned_traj = np.empty((T+1, nx))
    planned_traj[0, :] = x_init
    
    for k in range(T):
        planned_traj[k+1, :] = dynamics_func(planned_traj[k, :], u_opt[k, :], dt)
    
    return planned_traj
