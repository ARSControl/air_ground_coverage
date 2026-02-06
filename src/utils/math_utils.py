"""
Cleaned utility functions for HEDAC.
"""

import numpy as np
from numba import njit, prange
from typing import Tuple, Optional
import warnings


# =============================================================================
# Normalization Functions
# =============================================================================


def normalize_to_pdf(
    mat: np.ndarray, map_array: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Normalize a matrix to sum to 1 (probability distribution).

    Args:
        mat: Input matrix
        map_array: Optional binary map to mask obstacles

    Returns:
        Normalized matrix
    """
    if map_array is not None:
        masked = mat * (map_array == 0)
        return masked / (np.sum(masked) + 1e-10)
    return mat / (np.sum(mat) + 1e-10)


def min_max_normalize(mat: np.ndarray) -> np.ndarray:
    """
    Normalize matrix to [0, 1] range.

    Args:
        mat: Input matrix

    Returns:
        Normalized matrix in [0, 1]
    """
    min_val = np.min(mat)
    max_val = np.max(mat)
    if max_val - min_val < 1e-10:
        return np.zeros_like(mat)
    return (mat - min_val) / (max_val - min_val + 1e-10)


# =============================================================================
# Heat Equation Functions
# =============================================================================
def offset(mat, i, j):
    """
    offset a 2D matrix by i, j
    """
    rows, cols = mat.shape
    rows = rows - 2
    cols = cols - 2
    return mat[1 + i : 1 + i + rows, 1 + j : 1 + j + cols]


def compute_laplacian(temperature: np.ndarray, alpha: float) -> np.ndarray:
    """
    Compute the Laplacian of a 2D matrix using finite differences.

    Args:
        temperature: Input 2D matrix
        alpha: Heat diffusion coefficient

    Returns:
        Laplacian matrix
    """
    laplacian = (
        alpha * offset(temperature, 1, 0)
        + alpha * offset(temperature, -1, 0)
        + alpha * offset(temperature, 0, 1)
        + alpha * offset(temperature, 0, -1)
        - 4 * offset(temperature, 0, 0)
    )
    return laplacian


def update_heat(
    heat: np.ndarray,
    source: np.ndarray,
    map_array: np.ndarray,
    local_cooling_matrix: np.ndarray,
    dt: float,
    alpha: float,
    source_strength: float,
    beta: float,
    local_cooling: float,
    dx: float,
) -> np.ndarray:
    """
    Update heat field using the heat equation.

    Args:
        heat: Current heat field (H, W)
        source: Source term field (H, W)
        map_array: Binary map where 1=obstacle, 0=free
        local_cooling_matrix: Local cooling contributions (H, W)
        dt: Time step
        alpha: Heat diffusion coefficient
        source_strength: Source term scaling
        beta: Heat decay coefficient
        local_cooling: Local cooling coefficient
        dx: Spatial resolution

    Returns:
        Updated heat field
    """
    new_temperature = np.copy(heat)

    # Compute Laplacian
    laplacian = compute_laplacian(heat, alpha)

    # Update heat equation for free cells
    # Standard heat equation: du/dt = alpha * laplacian(u) + source - cooling
    new_temperature[1:-1, 1:-1] += dt * (
        (laplacian / (dx * dx))
        + source_strength * offset(source, 0, 0)
        - local_cooling * offset(local_cooling_matrix, 0, 0)
        - beta * offset(heat, 0, 0)
    )

    return new_temperature


@njit(cache=True)
def roll_optimized(arr: np.ndarray, shift: int, axis: int) -> np.ndarray:
    """
    Efficient roll implementation using slicing.

    Args:
        arr: Input array
        shift: Number of positions to shift
        axis: Axis along which to shift (0 or 1)

    Returns:
        Rolled array
    """
    result = np.empty_like(arr)
    if axis == 0:
        shift = shift % arr.shape[0]
        if shift > 0:
            result[:shift] = arr[-shift:]
            result[shift:] = arr[:-shift]
        else:
            result[:] = arr
    elif axis == 1:
        shift = shift % arr.shape[1]
        if shift > 0:
            result[:, :shift] = arr[:, -shift:]
            result[:, shift:] = arr[:, :-shift]
        else:
            result[:] = arr
    return result


@njit(parallel=True, cache=True, fastmath=True)
def update_heat_optimized(
    heat: np.ndarray,
    source: np.ndarray,
    map_array: np.ndarray,
    local_cooling_matrix: np.ndarray,
    dt: float,
    alpha: float,
    source_strength: float,
    beta: float,
    local_cooling: float,
    dx: float,
) -> np.ndarray:
    """
    Optimized heat equation update with parallelization.

    Updates the heat field using the heat equation:
    dT/dt = alpha * Laplacian(T) + source_strength * source - beta * T - local_cooling * cooling_matrix

    Args:
        heat: Current heat field (H, W)
        source: Source term field (H, W)
        map_array: Binary map where 1=obstacle, 0=free
        local_cooling_matrix: Local cooling contributions (H, W)
        dt: Time step
        alpha: Heat diffusion coefficient
        source_strength: Source term scaling
        beta: Heat decay coefficient
        local_cooling: Local cooling coefficient
        dx: Spatial resolution

    Returns:
        Updated heat field
    """
    n, m = heat.shape
    new_temperature = np.empty_like(heat)

    # Compute Laplacian using 5-point stencil
    laplacian = (
        roll_optimized(heat, 1, axis=0)
        + roll_optimized(heat, -1, axis=0)
        + roll_optimized(heat, 1, axis=1)
        + roll_optimized(heat, -1, axis=1)
        - 4 * heat
    ) / (dx * dx)

    # Update heat equation for free cells
    for i in prange(n):
        for j in range(m):
            if map_array[i, j] == 0:  # Free cell
                new_temperature[i, j] = heat[i, j] + dt * (
                    alpha * laplacian[i, j]
                    + source_strength * source[i, j]
                    - beta * heat[i, j]
                    - local_cooling * local_cooling_matrix[i, j]
                )
            else:  # Obstacle - keep original value (acts as boundary condition)
                new_temperature[i, j] = heat[i, j]

    return new_temperature


# =============================================================================
# Coverage Functions
# =============================================================================


def rbf_kernel(center: np.ndarray, x: np.ndarray, eps: float) -> float:
    """
    Radial basis function with Gaussian kernel.

    Args:
        center: Center point
        x: Query point
        eps: Shape parameter (1/radius)

    Returns:
        RBF value
    """
    d = center - x
    return np.exp(-eps * np.dot(d, d))


def create_agent_block(nb_var: int, min_val: float, agent_radius: float) -> np.ndarray:
    """
    Create a coverage block representing the agent's footprint.

    Uses RBF with Gaussian kernel where the block extends until the RBF
    value drops below min_val.

    Args:
        nb_var: Number of variables (dimensions)
        min_val: Minimum RBF value threshold
        agent_radius: Agent radius (determines RBF spread)

    Returns:
        2D coverage block matrix
    """
    eps = 1.0 / agent_radius
    l2_sqrd = -np.log(min_val) / eps
    l2_sqrd_single = l2_sqrd / nb_var
    l2_single = np.sqrt(l2_sqrd_single)

    # Round up to nearest integer
    l2_upper = int(np.ceil(l2_single))

    # Create symmetric block
    num_rows = l2_upper * 2 + 1
    num_cols = num_rows
    block = np.zeros((num_rows, num_cols))
    center = np.array([num_rows // 2, num_cols // 2])

    for i in range(num_rows):
        for j in range(num_cols):
            block[i, j] = rbf_kernel(center, np.array([j, i]), eps)

    return block


def clamp_kernel_1d(
    pos: int, low_lim: int, high_lim: int, kernel_size: int
) -> Tuple[slice, int, int]:
    """
    Calculate indices for kernel placement on grid with boundary handling.

    Args:
        pos: Center position
        low_lim: Lower grid boundary
        high_lim: Upper grid boundary
        kernel_size: Size of kernel

    Returns:
        Tuple of (grid_slice, kernel_start, num_kernel_elements)
    """
    half_k = kernel_size // 2

    # Clamp position to valid grid range
    pos = max(low_lim, min(pos, high_lim - 1))

    start_grid = pos - half_k
    start_kernel = 0
    num_kernel = kernel_size

    # Handle left/bottom boundary clipping
    if start_grid < low_lim:
        overflow = low_lim - start_grid
        start_kernel = overflow
        num_kernel = kernel_size - overflow
        start_grid = low_lim

    # Handle right/top boundary clipping
    if start_grid + num_kernel > high_lim:
        overflow = (start_grid + num_kernel) - high_lim
        num_kernel = num_kernel - overflow

    # Safety check: ensure num_kernel is valid
    if num_kernel <= 0:
        # Return empty slice - but this shouldn't happen with position clamping
        return slice(start_grid, start_grid), 0, 0

    grid_slice = slice(start_grid, start_grid + num_kernel)

    return grid_slice, start_kernel, num_kernel


def update_coverage_from_trajectory(
    trajectory: np.ndarray,
    coverage_density: np.ndarray,
    coverage_block: np.ndarray,
    map_shape: Tuple[int, int],
    resolution: float,
    map_limits: Tuple[float, float, float, float],
) -> np.ndarray:
    """
    Update coverage density from agent trajectory.

    Args:
        trajectory: Array of positions (T, 2)
        coverage_density: Current coverage density map
        coverage_block: Agent coverage block
        map_shape: Shape of map (height, width)
        resolution: Spatial resolution
        map_limits: (x_min, x_max, y_min, y_max)

    Returns:
        Updated coverage density
    """
    x_min, x_max, y_min, y_max = map_limits
    n_x, n_y = map_shape[1], map_shape[0]  # Note: numpy is (rows, cols) = (y, x)
    half_block = coverage_block.shape[0] // 2

    # Convert positions to grid indices
    grid_x = ((trajectory[:, 0] - x_min) / resolution).astype(int)
    grid_y = ((trajectory[:, 1] - y_min) / resolution).astype(int)

    # Clip to valid range
    grid_x = np.clip(grid_x, 0, n_x - 1)
    grid_y = np.clip(grid_y, 0, n_y - 1)

    for t in range(len(trajectory)):
        gx, gy = grid_x[t], grid_y[t]

        # Calculate block placement
        x_start = max(gx - half_block, 0)
        x_end = min(gx + half_block + 1, n_x)
        y_start = max(gy - half_block, 0)
        y_end = min(gy + half_block + 1, n_y)

        # Calculate block indices
        block_x_start = half_block - (gx - x_start)
        block_x_end = half_block + (x_end - gx)
        block_y_start = half_block - (gy - y_start)
        block_y_end = half_block + (y_end - gy)

        # Add coverage
        coverage_density[y_start:y_end, x_start:x_end] += coverage_block[
            block_y_start:block_y_end, block_x_start:block_x_end
        ]

    return coverage_density


# =============================================================================
# Field of View Functions
# =============================================================================


def init_fov(fov_deg: float, fov_depth: float) -> np.ndarray:
    """
    Initialize field of view polygon at origin.

    Creates a triangular FOV with tip at origin.

    Args:
        fov_deg: Field of view angle in degrees
        fov_depth: Depth of field of view

    Returns:
        FOV polygon vertices (3, 2)
    """
    fov_rad = np.radians(fov_deg)
    left_angle = fov_rad / 2
    right_angle = -fov_rad / 2

    left_point = [fov_depth * np.cos(left_angle), fov_depth * np.sin(left_angle)]
    right_point = [fov_depth * np.cos(right_angle), fov_depth * np.sin(right_angle)]

    return np.array([[0, 0], left_point, right_point])


def rotate_and_translate(
    points: np.ndarray, pos: np.ndarray, theta: float
) -> np.ndarray:
    """
    Rotate and translate points.

    Args:
        points: Points to transform (N, 2)
        pos: Translation vector (2,)
        theta: Rotation angle in radians

    Returns:
        Transformed points
    """
    rot_matrix = np.array(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
    )
    return np.dot(points, rot_matrix.T) + pos


def bilinear_interpolate(grid: np.ndarray, pos: np.ndarray) -> float:
    """
    Bilinear interpolation on 2D grid.

    Args:
        grid: 2D grid values
        pos: Position to interpolate (x, y)

    Returns:
        Interpolated value
    """
    x, y = pos
    x0, y0 = int(x), int(y)
    x1, y1 = min(x0 + 1, grid.shape[1] - 1), min(y0 + 1, grid.shape[0] - 1)

    # Handle boundaries
    x0 = max(0, min(x0, grid.shape[1] - 1))
    y0 = max(0, min(y0, grid.shape[0] - 1))

    # Bilinear interpolation
    dx = x - x0
    dy = y - y0

    c00 = grid[y0, x0]
    c01 = grid[y0, x1]
    c10 = grid[y1, x0]
    c11 = grid[y1, x1]

    return (
        (1 - dx) * (1 - dy) * c00
        + dx * (1 - dy) * c01
        + (1 - dx) * dy * c10
        + dx * dy * c11
    )


# =============================================================================
# Distribution Functions
# =============================================================================


def gauss_pdf(
    points: np.ndarray, mean: np.ndarray, covariance: np.ndarray
) -> np.ndarray:
    """
    Compute multivariate Gaussian PDF at given points.

    Args:
        points: Points to evaluate (N, 2)
        mean: Mean vector (2,)
        covariance: Covariance matrix (2, 2)

    Returns:
        PDF values at points (N,)
    """
    inv_cov = np.linalg.inv(covariance)
    det_cov = np.linalg.det(covariance)

    diff = points - mean
    exponent = -0.5 * np.sum(diff @ inv_cov * diff, axis=1)
    coefficient = 1 / np.sqrt((2 * np.pi) ** 2 * det_cov)

    return coefficient * np.exp(exponent)


def create_gaussian_goal_density(
    grid_points: np.ndarray,
    means: np.ndarray,
    covariances: np.ndarray,
    map_shape: Tuple[int, int],
    map_array: np.ndarray,
) -> np.ndarray:
    """
    Create goal density from mixture of Gaussians.

    Args:
        grid_points: Grid coordinates (N, 2)
        means: Gaussian means (K, 2)
        covariances: Gaussian covariances (K, 2, 2)
        map_shape: Shape of output map (H, W)
        map_array: Binary map for masking

    Returns:
        Goal density map
    """
    density = np.zeros(len(grid_points))

    for mean, cov in zip(means, covariances):
        density += gauss_pdf(grid_points, mean, cov)

    # Normalize
    density = min_max_normalize(density)

    # Reshape and apply map mask
    density_map = density.reshape(map_shape)
    density_map = density_map * (map_array == 0)

    # Renormalize after masking
    return normalize_to_pdf(density_map, map_array)


# =============================================================================
# Gradient Computation
# =============================================================================


def compute_gradient_direction(
    heat_field: np.ndarray,
    agent_pos: np.ndarray,
    map_array: np.ndarray,
    wall_avoidance_weight: float = 0.5,
    kernel_radius: int = 3,
    boundary_gradient: float = 1.0,
) -> np.ndarray:
    """
    Compute gradient direction for agent movement with wall and boundary avoidance.

    Args:
        heat_field: Current heat field
        agent_pos: Agent position (x, y)
        map_array: Binary map
        wall_avoidance_weight: Weight for wall avoidance term
        kernel_radius: Radius for obstacle detection
        boundary_gradient: Strength of boundary repulsion force

    Returns:
        Gradient direction vector (2,)
    """
    # Compute heat gradient
    # With transpose: heat_field.T has shape (cols, rows), so np.gradient returns (d/dx, d/dy)
    # Therefore we assign: grad_x, grad_y (swapped order!)
    grad_x, grad_y = np.gradient(heat_field.T)
    # Transpose back to (rows, cols) format for bilinear_interpolate
    grad_x = grad_x.T
    grad_y = grad_y.T

    # Interpolate gradient at agent position
    x, y = agent_pos.astype(int)
    height, width = map_array.shape

    if 0 <= x < heat_field.shape[1] and 0 <= y < heat_field.shape[0]:
        gradient = np.array(
            [
                bilinear_interpolate(grad_x, agent_pos),
                bilinear_interpolate(grad_y, agent_pos),
            ]
        )
    else:
        gradient = np.zeros(2)

    # Add wall avoidance
    wall_effect = np.zeros(2)
    for dx in range(-kernel_radius, kernel_radius + 1):
        for dy in range(-kernel_radius, kernel_radius + 1):
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_array.shape[1] and 0 <= ny < map_array.shape[0]:
                if map_array[ny, nx] == 1:  # Obstacle
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist > 0:
                        influence = np.exp(-(dist**2) / (2 * kernel_radius**2))
                        wall_effect += influence * np.array([-dx, -dy]) / dist

    # Combine gradient and wall avoidance
    gradient += 0.0 * wall_effect

    # Add boundary enforcement to keep agents inside the map
    # Push agents away from map boundaries
    boundary_margin = kernel_radius
    if y <= boundary_margin:
        # Near top boundary, push down
        gradient[1] += boundary_gradient
    elif y >= height - boundary_margin - 1:
        # Near bottom boundary, push up
        gradient[1] -= boundary_gradient
    if x <= boundary_margin:
        # Near left boundary, push right
        gradient[0] += boundary_gradient
    elif x >= width - boundary_margin - 1:
        # Near right boundary, push left
        gradient[0] -= boundary_gradient

    # Normalize
    norm = np.linalg.norm(gradient)
    if norm > 0:
        gradient /= norm

    return gradient


def compute_gradient_direction_v2(
    gradient_x: np.ndarray,
    gradient_y: np.ndarray,
    agent_pos: np.ndarray,
    map_array: np.ndarray,
    heading: float = 0.0,
    wall_avoidance_weight: float = 0.5,
    kernel_radius: int = 3,
) -> np.ndarray:
    """
    Compute gradient direction for agent movement with wall and boundary avoidance.
    Based on calculate_gradient_map_v2 from the old implementation.

    Args:
        gradient_x: Pre-computed x gradient of heat field
        gradient_y: Pre-computed y gradient of heat field
        agent_pos: Agent position (x, y)
        map_array: Binary map
        heading: Agent heading angle in radians
        wall_avoidance_weight: Weight for wall avoidance term
        kernel_radius: Radius for obstacle detection

    Returns:
        Gradient direction vector (2,)
    """
    x, y = agent_pos.astype(int)
    heading_vector = np.array([np.cos(heading), np.sin(heading)])
    gradient = np.zeros(2)
    height, width = map_array.shape

    # Interpolate gradient at agent position using bilinear interpolation
    if 0 <= x < width and 0 <= y < height:
        gradient[0] = bilinear_interpolate(gradient_x, agent_pos)
        gradient[1] = bilinear_interpolate(gradient_y, agent_pos)

    # Calculate the wall avoidance effect based on nearby obstacles
    wall_effect = np.zeros(2)

    # Generate a grid of relative coordinates within the kernel
    dx_grid, dy_grid = np.meshgrid(
        np.arange(-kernel_radius, kernel_radius + 1),
        np.arange(-kernel_radius, kernel_radius + 1),
        indexing="ij",
    )

    # Calculate absolute positions
    next_x = x + dx_grid
    next_y = y + dy_grid

    # Mask valid positions within bounds
    valid_mask = (0 <= next_x) & (next_x < width) & (0 <= next_y) & (next_y < height)

    # Mask positions corresponding to obstacles
    obstacle_mask = map_array[next_y[valid_mask], next_x[valid_mask]] == 1

    # Compute distances for valid positions
    dx_valid = dx_grid[valid_mask]
    dy_valid = dy_grid[valid_mask]
    # Compute distances and avoid division by zero
    distances = np.sqrt(dx_valid**2 + dy_valid**2)
    distances[distances == 0] = np.inf  # Prevent division by zero

    # Compute influence for obstacles
    influence = np.exp(-(distances**2) / (2 * kernel_radius**2))

    # Compute direction away from obstacles
    direction_x = -dx_valid / distances
    direction_y = -dy_valid / distances

    # Apply mask to influence and direction
    direction_x = direction_x * obstacle_mask
    direction_y = direction_y * obstacle_mask
    influence = influence * obstacle_mask

    # Sum up contributions
    wall_effect[0] = np.sum(influence * direction_x)
    wall_effect[1] = np.sum(influence * direction_y)

    # Decompose wall effect into parallel and perpendicular components
    parallel_effect = np.dot(wall_effect, heading_vector) * heading_vector
    perpendicular_effect = wall_effect - parallel_effect

    # Scale perpendicular effect to reduce sharp turns
    perpendicular_scaling = 0.5  # Adjust sensitivity
    wall_effect = (
        1 - perpendicular_scaling
    ) * parallel_effect + perpendicular_scaling * perpendicular_effect

    # Combine interpolated gradient and wall effect
    gradient += wall_effect * wall_avoidance_weight

    # Normalize the resulting gradient to prevent erratic movements
    norm = np.linalg.norm(gradient)
    if norm > 0:
        gradient /= norm

    return gradient


def calculate_gradient(
    gradient_x: np.ndarray,
    gradient_y: np.ndarray,
    agent_pos: np.ndarray,
    map_array: np.ndarray,
    heading: float = 0.0,
    wall_avoidance_weight: float = 0.5,
    kernel_radius: int = 3,
) -> np.ndarray:
    """
    Calculate movement direction of the agent by considering the gradient
    of the temperature field near the agent.

    This is the simplified original HEDAC gradient function that:
    1. Interpolates the heat gradient at the agent position
    2. Adds boundary repulsion when near map edges

    Note: This function does NOT include wall avoidance for obstacles.
    For maps with obstacles, use compute_gradient_direction_v2 or calculate_gradient_map.
    """
    # Width is x axis, Height is y axis
    x, y = agent_pos.astype(int)
    # note x axis corresponds to col and y axis corresponds to row
    col, row = x, y
    height, width = map_array.shape

    gradient = np.zeros(2)
    # if agent is inside the grid, interpolate the gradient for agent position
    if row > 0 and row < height - 1 and col > 0 and col < width - 1:
        gradient[0] = bilinear_interpolate(gradient_x, agent_pos)
        gradient[1] = bilinear_interpolate(gradient_y, agent_pos)

    # if kernel around the agent is outside the grid,
    # use the gradient to direct the agent inside the grid
    boundary_gradient = 10.0
    pad = kernel_radius - 1
    if row <= pad:
        gradient[1] = boundary_gradient
    elif row >= height - 1 - pad:
        gradient[1] = -boundary_gradient

    if col <= pad:
        gradient[0] = boundary_gradient
    elif col >= width - 1 - pad:
        gradient[0] = -boundary_gradient

    return gradient
