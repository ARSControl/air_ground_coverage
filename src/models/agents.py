"""
Agent models for HEDAC.
"""

import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass, field


@dataclass
class AgentState:
    """State of a double integrator agent."""

    position: np.ndarray  # (x, y)
    velocity: np.ndarray  # (vx, vy)
    heading: float  # theta in radians
    angular_velocity: float  # omega

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
        self.position_history: List[np.ndarray] = [self.position.copy()]
        self.heading_history: List[float] = [self.heading]

        # Sensing
        self.sens_range: float = 10.0
        self.fov_edges: Optional[np.ndarray] = None
        self.neighbors: List[int] = []

        # Internal state
        self.grad: np.ndarray = np.zeros(2)

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
                forward_vel * forward_dir + 0.1 * lateral_vel * lateral_dir
            )
        else:
            target_vel_body = target_velocity

        velocity_error = target_vel_body - self.velocity
        k_p_vel = 1.0
        acceleration = k_p_vel * velocity_error

        self.step(acceleration, angular_accel)

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


class AgentTeam:
    """Team of agents."""

    def __init__(self, agents: List[DoubleIntegratorAgent]):
        """
        Initialize team.

        Args:
            agents: List of agents
        """
        self.agents = agents
        self.adjacency_matrix = np.eye(len(agents))

    def __len__(self) -> int:
        return len(self.agents)

    def __getitem__(self, idx: int) -> DoubleIntegratorAgent:
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
