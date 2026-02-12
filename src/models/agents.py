"""
Agent models for HEDAC.
"""

import numpy as np
from numpy.typing import NDArray
from typing import Tuple, Optional, List, Protocol
from dataclasses import dataclass, field


@dataclass
class AgentState:
    """State of a double integrator agent."""

    position: NDArray[np.float64]  # (x, y)
    velocity: NDArray[np.float64]  # (vx, vy)
    heading: float  # theta in radians
    angular_velocity: float  # omega
    observations_history: NDArray[np.float64] = field(
        default_factory=lambda: np.empty((0, 2))
    )  # History of sensed observations

    def to_array(self) -> np.ndarray:
        """Convert to numpy array [x, y, theta, vx, vy, omega]."""
        return np.array(
            [
                self.position[0],
                self.position[1],
                self.heading,
                self.velocity[0],
                self.velocity[1],
                self.angular_velocity,
            ]
        )

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "AgentState":
        """Create from numpy array."""
        return cls(
            position=arr[:2], velocity=arr[3:5], heading=arr[2], angular_velocity=arr[5]
        )


class DoubleIntegratorAgent:
    """
    Double integrator agent with heading control.

    State: [x, y, theta, vx, vy, omega]
    - Position (x, y)
    - Heading theta
    - Velocity (vx, vy)
    - Angular velocity omega
    """

    def __init__(
        self,
        x0: np.ndarray,
        theta0: float,
        max_dx: float = 1.0,
        max_ddx: float = 0.5,
        max_dtheta: float = np.pi / 4,
        max_ddtheta: float = np.pi / 8,
        dt: float = 0.1,
        agent_id: int = 0,
        observations_range: int = 10,
        observations_count: int = 10,
        sens_range: float = 10.0,
    ):
        """
        Initialize agent.

        Args:
            x0: Initial position [x, y]
            theta0: Initial heading in radians
            max_dx: Maximum velocity
            max_ddx: Maximum acceleration
            max_dtheta: Maximum angular velocity
            max_ddtheta: Maximum angular acceleration
            dt: Time step
            agent_id: Unique identifier
            observations_range: Range of observations
            observations_count: Number of observations to take at each sensing step
            sens_range: Sensing range for the agent (used for neighbor detection)
        """
        self.id = agent_id
        self.position = np.array(x0, dtype=float)
        self.heading = theta0
        self.velocity = np.zeros(2)
        self.angular_velocity = 0.0

        # Limits
        self.max_dx = max_dx
        self.max_ddx = max_ddx
        self.max_dtheta = max_dtheta
        self.max_ddtheta = max_ddtheta
        self.dt = dt

        # History
        self.position_history: List[NDArray[np.float64]] = [self.position.copy()]
        self.heading_history: List[float] = [self.heading]

        # Sensing
        self.sens_range = sens_range
        self.fov_edges: Optional[np.ndarray] = None
        self.neighbors: List[int] = []
        self.observations_history: NDArray[np.float64] = np.empty(
            (0, 2)
        )  # History of sensed positions
        self._observations_count = observations_count
        self._observations_range = observations_range

        # GP observation storage
        self.all_observations: NDArray[np.float64] = np.empty((0, 3))

        # Internal state
        self.grad: NDArray[np.float64] = np.zeros(2)

    @property
    def x(self) -> np.ndarray:
        """Current position."""
        return self.position

    @property
    def theta(self) -> float:
        """Current heading."""
        return self.heading

    @property
    def x_hist(self) -> np.ndarray:
        """Position history as array."""
        return np.array(self.position_history)

    @property
    def trajectory(self) -> np.ndarray:
        """Full trajectory (position and heading) history."""
        return np.array(
            [
                [pos[0], pos[1], head]
                for pos, head in zip(self.position_history, self.heading_history)
            ]
        )

    @property
    def observations_count(self) -> int:
        """Number of observations to take at each sensing step."""
        return self._observations_count

    @observations_count.setter
    def observations_count(self, count: int):
        """Set the number of observations to take at each sensing step."""
        self._observations_count = count

    @property
    def observations_range(self) -> int:
        """Range of observations."""
        return self._observations_range

    @observations_range.setter
    def observations_range(self, obs_range: int):
        """Set the range of observations."""
        self._observations_range = obs_range

    def step(self, acceleration: np.ndarray, angular_acceleration: float):
        """
        Update agent state with acceleration commands.

        Args:
            acceleration: Linear acceleration [ax, ay]
            angular_acceleration: Angular acceleration
        """
        # Clip accelerations
        acc_norm = np.linalg.norm(acceleration)
        if acc_norm > self.max_ddx:
            acceleration = acceleration / acc_norm * self.max_ddx

        angular_acceleration = np.clip(
            angular_acceleration, -self.max_ddtheta, self.max_ddtheta
        )

        # Update velocities
        self.velocity += acceleration * self.dt
        vel_norm = np.linalg.norm(self.velocity)
        if vel_norm > self.max_dx:
            self.velocity = self.velocity / vel_norm * self.max_dx

        self.angular_velocity += angular_acceleration * self.dt
        self.angular_velocity = np.clip(
            self.angular_velocity, -self.max_dtheta, self.max_dtheta
        )

        # Update position and heading
        self.position += self.velocity * self.dt
        self.heading += self.angular_velocity * self.dt

        # Normalize heading to [-pi, pi]
        self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))

        # Store history
        self.position_history.append(self.position.copy())
        self.heading_history.append(self.heading)

    def clip_position(self, x_min: float, x_max: float, y_min: float, y_max: float):
        """
        Clip agent position to stay within map boundaries.

        Args:
            x_min: Minimum x coordinate
            x_max: Maximum x coordinate
            y_min: Minimum y coordinate
            y_max: Maximum y coordinate
        """
        self.position[0] = np.clip(self.position[0], x_min, x_max)
        self.position[1] = np.clip(self.position[1], y_min, y_max)

    def track_velocity_and_heading(
        self,
        target_velocity: np.ndarray,
        target_heading: float,
        penalize_lateral: bool = True,
    ):
        """
        Track target velocity and heading using simple controller.

        Args:
            target_velocity: Desired velocity vector [vx, vy]
            target_heading: Desired heading angle
            penalize_lateral: Whether to penalize lateral velocity
        """
        # Compute heading error
        heading_error = target_heading - self.heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Simple PD control for angular velocity
        k_p_angular = 2.0
        angular_accel = k_p_angular * heading_error

        # Compute velocity error
        if penalize_lateral:
            # Decompose velocity into forward and lateral components
            forward_dir = np.array([np.cos(self.heading), np.sin(self.heading)])
            forward_vel = np.dot(target_velocity, forward_dir)

            lateral_dir = np.array([-np.sin(self.heading), np.cos(self.heading)])
            lateral_vel = np.dot(target_velocity, lateral_dir)

            # Target velocity in body frame
            target_vel_body = (
                forward_vel * forward_dir + 0.01 * lateral_vel * lateral_dir
            )
        else:
            target_vel_body = target_velocity

        velocity_error = target_vel_body - self.velocity
        k_p_vel = 1.0
        acceleration = k_p_vel * velocity_error

        self.step(acceleration, angular_accel)

    def sense_environment(self) -> np.ndarray:
        """
        Sense the environment within the sensing range.

        Returns:
            Array of sensed points (shape: [n_points, 2])
        """
        angles = np.linspace(0, 2 * np.pi, self.observations_count, endpoint=False)
        distances = np.random.uniform(
            0, self.observations_range, size=self.observations_count
        )
        obs_x = self.position[0] + distances * np.cos(angles)
        obs_y = self.position[1] + distances * np.sin(angles)
        obs = np.column_stack((obs_x, obs_y))
        self.observations_history = np.vstack((self.observations_history, obs))
        return obs

    def sense_environment_gp(
        self,
        true_density_map: np.ndarray,
        noise_std: float = 0.05,
        n_samples: Optional[int] = None,
        max_resample: int = 50,
    ) -> np.ndarray:
        """
        Sample observations from the true density map with noise.

        Args:
            true_density_map: Ground truth density map (H, W)
            noise_std: Standard deviation of Gaussian observation noise
            n_samples: Number of samples per step (defaults to observations_count)
            max_resample: Max attempts to keep samples within bounds

        Returns:
            observations: (N, 3) array [x, y, noisy_value]
        """
        if n_samples is None:
            n_samples = self.observations_count

        height, width = true_density_map.shape
        angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
        distances = np.random.uniform(0, self.observations_range, size=n_samples)

        obs = []
        for angle, dist in zip(angles, distances):
            x = self.position[0] + dist * np.cos(angle)
            y = self.position[1] + dist * np.sin(angle)

            # Resample if out of bounds
            attempts = 0
            while (
                x < 0 or x >= width or y < 0 or y >= height
            ) and attempts < max_resample:
                dist = np.random.uniform(0, self.observations_range)
                x = self.position[0] + dist * np.cos(angle)
                y = self.position[1] + dist * np.sin(angle)
                attempts += 1

            x = np.clip(x, 0, width - 1)
            y = np.clip(y, 0, height - 1)

            # Nearest-neighbor sampling
            value = true_density_map[int(round(y)), int(round(x))]
            noisy_value = value + np.random.normal(0.0, noise_std)
            obs.append([x, y, noisy_value])

        observations = np.array(obs, dtype=float)
        if observations.size > 0:
            self.all_observations = np.vstack((self.all_observations, observations))
        return observations

    def get_state(self) -> AgentState:
        """Get current state."""
        return AgentState(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            heading=self.heading,
            angular_velocity=self.angular_velocity,
        )

    def set_state(self, state: AgentState):
        """Set current state."""
        self.position = state.position.copy()
        self.velocity = state.velocity.copy()
        self.heading = state.heading
        self.angular_velocity = state.angular_velocity


class AgentLike(Protocol):
    position: NDArray[np.float64]
    neighbors: List[int]
    theta: float

    @property
    def x_hist(self) -> np.ndarray: ...

    def sense_environment(self) -> np.ndarray: ...

    def sense_environment_gp(
        self,
        true_density_map: np.ndarray,
        noise_std: float = 0.05,
        n_samples: Optional[int] = None,
        max_resample: int = 50,
    ) -> np.ndarray: ...

    def track_velocity_and_heading(
        self,
        target_velocity: np.ndarray,
        target_heading: float,
        penalize_lateral: bool = True,
    ): ...

    def clip_position(self, x_min: float, x_max: float, y_min: float, y_max: float): ...


class DubinsAgent:
    """
    Dubins model for fixed-wing aircraft in level flight.

    State: [x, y, theta]
    - Position (x, y)
    - Heading theta

    Dynamics:
    x_dot = v * cos(theta)
    y_dot = v * sin(theta)
    theta_dot = u, |u| <= max_turn_rate
    """

    def __init__(
        self,
        x0: np.ndarray,
        theta0: float,
        forward_speed: float = 5.0,
        max_bank_angle: float = 30.0,
        dt: float = 0.1,
        agent_id: int = 0,
        observations_range: int = 10,
        observations_count: int = 10,
        sens_range: float = 10.0,
    ):
        """
        Initialize Dubins agent.

        Args:
            x0: Initial position [x, y]
            theta0: Initial heading in radians
            forward_speed: Constant forward speed (m/s)
            max_bank_angle: Maximum bank angle in degrees
            dt: Time step
            agent_id: Unique identifier
            observations_range: Range of observations
            observations_count: Number of observations to take at each sensing step
            sens_range: Sensing range for the agent (used for neighbor detection)
        """
        self.id = agent_id
        self.position = np.array(x0, dtype=float)
        self.heading = theta0
        self.forward_speed = float(forward_speed)
        self.max_bank_angle = float(max_bank_angle)
        self.dt = dt

        # Convert bank angle to max turn rate
        g = 9.81
        bank_angle_rad = np.deg2rad(self.max_bank_angle)
        if self.forward_speed <= 0:
            raise ValueError("forward_speed must be positive for DubinsAgent")
        if self.max_bank_angle <= 0:
            raise ValueError("max_bank_angle must be positive for DubinsAgent")
        self.max_turn_rate = (g * np.tan(bank_angle_rad)) / self.forward_speed

        # History
        self.position_history: List[NDArray[np.float64]] = [self.position.copy()]
        self.heading_history: List[float] = [self.heading]

        # Sensing
        self.sens_range = sens_range
        self.fov_edges: Optional[np.ndarray] = None
        self.neighbors: List[int] = []
        self.observations_history: NDArray[np.float64] = np.empty((0, 2))
        self._observations_count = observations_count
        self._observations_range = observations_range

        # GP observation storage
        self.all_observations: NDArray[np.float64] = np.empty((0, 3))

        # Internal state
        self.grad: NDArray[np.float64] = np.zeros(2)

    @property
    def x(self) -> np.ndarray:
        """Current position."""
        return self.position

    @property
    def theta(self) -> float:
        """Current heading."""
        return self.heading

    @property
    def x_hist(self) -> np.ndarray:
        """Position history as array."""
        return np.array(self.position_history)

    @property
    def trajectory(self) -> np.ndarray:
        """Full trajectory (position and heading) history."""
        return np.array(
            [
                [pos[0], pos[1], head]
                for pos, head in zip(self.position_history, self.heading_history)
            ]
        )

    @property
    def observations_count(self) -> int:
        """Number of observations to take at each sensing step."""
        return self._observations_count

    @observations_count.setter
    def observations_count(self, count: int):
        """Set the number of observations to take at each sensing step."""
        self._observations_count = count

    @property
    def observations_range(self) -> int:
        """Range of observations."""
        return self._observations_range

    @observations_range.setter
    def observations_range(self, obs_range: int):
        """Set the range of observations."""
        self._observations_range = obs_range

    def step(self, turn_rate: float):
        """
        Update agent state with turn rate command.

        Args:
            turn_rate: Desired turn rate (rad/s)
        """
        turn_rate = np.clip(turn_rate, -self.max_turn_rate, self.max_turn_rate)

        self.heading += turn_rate * self.dt
        self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))

        self.position[0] += self.forward_speed * np.cos(self.heading) * self.dt
        self.position[1] += self.forward_speed * np.sin(self.heading) * self.dt

        self.position_history.append(self.position.copy())
        self.heading_history.append(self.heading)

    def clip_position(self, x_min: float, x_max: float, y_min: float, y_max: float):
        """
        Clip agent position to stay within map boundaries.

        Args:
            x_min: Minimum x coordinate
            x_max: Maximum x coordinate
            y_min: Minimum y coordinate
            y_max: Maximum y coordinate
        """
        self.position[0] = np.clip(self.position[0], x_min, x_max)
        self.position[1] = np.clip(self.position[1], y_min, y_max)

    def track_velocity_and_heading(
        self,
        target_velocity: np.ndarray,
        target_heading: float,
        penalize_lateral: bool = True,
    ):
        """
        Track target heading with coordinated turn at constant speed.

        Args:
            target_velocity: Desired velocity vector [vx, vy]
            target_heading: Desired heading angle
            penalize_lateral: Unused for Dubins agent (kept for interface compatibility)
        """
        if np.linalg.norm(target_velocity) > 1e-10:
            target_heading = np.arctan2(target_velocity[1], target_velocity[0])

        heading_error = target_heading - self.heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        k_p_angular = 2.0
        turn_rate = np.clip(
            k_p_angular * heading_error, -self.max_turn_rate, self.max_turn_rate
        )
        self.step(turn_rate)

    def sense_environment(self) -> np.ndarray:
        """
        Sense the environment within the sensing range.

        Returns:
            Array of sensed points (shape: [n_points, 2])
        """
        angles = np.linspace(0, 2 * np.pi, self.observations_count, endpoint=False)
        distances = np.random.uniform(
            0, self.observations_range, size=self.observations_count
        )
        obs_x = self.position[0] + distances * np.cos(angles)
        obs_y = self.position[1] + distances * np.sin(angles)
        obs = np.column_stack((obs_x, obs_y))
        self.observations_history = np.vstack((self.observations_history, obs))
        return obs

    def sense_environment_gp(
        self,
        true_density_map: np.ndarray,
        noise_std: float = 0.05,
        n_samples: Optional[int] = None,
        max_resample: int = 50,
    ) -> np.ndarray:
        """
        Sample observations from the true density map with noise.

        Args:
            true_density_map: Ground truth density map (H, W)
            noise_std: Standard deviation of Gaussian observation noise
            n_samples: Number of samples per step (defaults to observations_count)
            max_resample: Max attempts to keep samples within bounds

        Returns:
            observations: (N, 3) array [x, y, noisy_value]
        """
        if n_samples is None:
            n_samples = self.observations_count

        height, width = true_density_map.shape
        angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
        distances = np.random.uniform(0, self.observations_range, size=n_samples)

        obs = []
        for angle, dist in zip(angles, distances):
            x = self.position[0] + dist * np.cos(angle)
            y = self.position[1] + dist * np.sin(angle)

            attempts = 0
            while (
                x < 0 or x >= width or y < 0 or y >= height
            ) and attempts < max_resample:
                dist = np.random.uniform(0, self.observations_range)
                x = self.position[0] + dist * np.cos(angle)
                y = self.position[1] + dist * np.sin(angle)
                attempts += 1

            x = np.clip(x, 0, width - 1)
            y = np.clip(y, 0, height - 1)

            value = true_density_map[int(round(y)), int(round(x))]
            noisy_value = value + np.random.normal(0.0, noise_std)
            obs.append([x, y, noisy_value])

        observations = np.array(obs, dtype=float)
        if observations.size > 0:
            self.all_observations = np.vstack((self.all_observations, observations))
        return observations

    def get_state(self) -> AgentState:
        """Get current state."""
        velocity = np.array(
            [
                self.forward_speed * np.cos(self.heading),
                self.forward_speed * np.sin(self.heading),
            ]
        )
        return AgentState(
            position=self.position.copy(),
            velocity=velocity,
            heading=self.heading,
            angular_velocity=0.0,
        )

    def set_state(self, state: AgentState):
        """Set current state."""
        self.position = state.position.copy()
        self.heading = state.heading


class AgentTeam:
    """Team of agents."""

    def __init__(self, agents: List[AgentLike]):
        """
        Initialize team.

        Args:
            agents: List of agents
        """
        self.agents = agents
        self.adjacency_matrix = np.eye(len(agents))

    def __len__(self) -> int:
        return len(self.agents)

    def __getitem__(self, idx: int) -> AgentLike:
        return self.agents[idx]

    def update_neighbors(self, map_array: np.ndarray, sens_range: float):
        """
        Update neighbor relationships based on sensing range.

        Args:
            map_array: Binary map
            sens_range: Sensing range
        """
        n = len(self.agents)
        for i in range(n):
            self.agents[i].neighbors = []
            for j in range(n):
                if i != j:
                    dist = np.linalg.norm(
                        self.agents[i].position - self.agents[j].position
                    )
                    if dist <= sens_range:
                        self.agents[i].neighbors.append(j)
                        self.adjacency_matrix[i, j] = 1

    def get_positions(self) -> np.ndarray:
        """Get all agent positions."""
        return np.array([agent.position for agent in self.agents])

    def get_histories(self) -> List[np.ndarray]:
        """Get position histories for all agents."""
        return [agent.x_hist for agent in self.agents]

    def sense_all_agents(self):
        """Have all agents sense the environment."""
        measurements = np.empty((0, 2))
        for agent in self.agents:
            obs = agent.sense_environment()
            measurements = np.vstack((measurements, obs))
        return measurements
