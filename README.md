# HEDAC: Heat Equation Driven Area Coverage

A cleaned and improved implementation of the HEDAC algorithm for multi-agent
ergodic control based on the paper by Ivić et al.

## Installation

This project uses [uv](https://github.com/astral-sh/uv) for dependency management.

### Using uv (recommended)

```bash
# Clone the repository
git clone <repository-url>
cd hedac

# Install dependencies
uv sync

# Activate the virtual environment
source .venv/bin/activate  # On Unix/macOS
# or
.venv\Scripts\activate  # On Windows
```

### Using pip

```bash
pip install -e .
```

## Quick Start

### Run with configuration file

```bash
# Run with default configuration
uv run python examples/run_hedac.py --config configs/default_params.yaml

# Run with single agent simple config
uv run python examples/run_hedac.py --config configs/single_agent_simple.yaml

# Run GP single-robot debugging example
uv run python examples/run_gp_single_robot.py --config configs/default_params.yaml
```

### Python API Usage

```python
import numpy as np
from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.models.agents import DoubleIntegratorAgent, DubinsAgent, AgentTeam

# Load or create map (obstacle-free 50x50 grid)
map_loader = MapLoader(size=[50, 50], resolution=1.0)
map_array = map_loader.load()

# Create goal density (Gaussian mixture)
height, width = map_array.shape
x = np.arange(width)
y = np.arange(height)
X, Y = np.meshgrid(x, y)
peak_x, peak_y = 25.0, 25.0
sigma = 10.0
goal_density = np.exp(-((X - peak_x)**2 + (Y - peak_y)**2) / (2 * sigma**2))
goal_density = goal_density / np.sum(goal_density)

# Load parameters from YAML config
params = HEDACParams.from_yaml("configs/default_params.yaml")

# Create agent
agent_model = params.get("agents.model_type", "double_integrator")
if agent_model == "dubins":
    dubins_config = params.get("agents.dubins", {})
    agent = DubinsAgent(
        x0=np.array([10.0, 10.0]),
        theta0=0.0,
        forward_speed=float(dubins_config.get("forward_speed", 5.0) or 5.0),
        max_bank_angle=float(dubins_config.get("max_bank_angle", 30.0) or 30.0),
        dt=params.dt_agent,
        agent_id=0,
    )
else:
    agent = DoubleIntegratorAgent(
        x0=np.array([10.0, 10.0]),
        theta0=0.0,
        max_dx=params.max_dx,
        max_ddx=params.max_ddx,
        max_dtheta=params.max_dtheta,
        max_ddtheta=params.max_ddtheta,
        dt=params.dt_agent,
        agent_id=0,
    )
agent_team = AgentTeam([agent])

# Initialize and run HEDAC
hedac = HEDACAlgorithm(params, map_loader, goal_density)
results = hedac.run(agent_team, num_steps=1000, verbose=True)

print(f"Final ergodic metric: {results['ergodic_metrics'][-1]:.4f}")
```

## GP-HEDAC Overview

This repo includes a centralized Gaussian Process (GP) model that aggregates
observations from all agents to build a posterior over the map. The mean
(exploitation) and normalized uncertainty (exploration) are combined into a
single goal density used by HEDAC at every step.

### Data Flow

1. Agents collect noisy samples of the underlying spatial field.
2. Samples are aggregated into a centralized dataset.
3. A fixed-hyperparameter GP computes posterior mean and std over the grid.
4. Mean + std are combined into a new goal density.
5. HEDAC uses this goal density as the source term for coverage.

### Combined Goal Density

We combine normalized mean and normalized uncertainty with:

```
combo = gamma * mean_norm + (1 - gamma) * std_norm
```

`gamma` controls exploration vs exploitation.

### Uncertainty Filtering (Optional)

When enabled, observations are filtered using normalized uncertainty:

- Keep new samples if uncertainty > `take_threshold`
- Remove old samples if uncertainty < `remove_threshold`

This keeps the dataset focused on regions that are still informative.

## GP Debug Visualization

Enable the 4-panel debugging visualization via:

```yaml
visualization:
  gp_debug: true
  gp_debug_interval: 50
  save_gp_frames: false
```

The debug view shows:

1. True goal density + robot trajectory + all observations + filtered dataset
2. Combined goal density (mean + std)
3. Coverage density
4. Ergodic metric over time

## Configuration (GP)

Key GP parameters in `configs/default_params.yaml`:

```yaml
gpr:
  implementation: "handcoded"
  length_scale: 5.0
  sigma_f: 1.0
  noise_level: 0.1
  gamma: 0.5
  obs_noise_std: 0.05
  obs_per_step: 10
  min_samples: 10
  use_filter: true
  take_threshold: 0.25
  remove_threshold: 0.15
  max_dataset_size: 500
  fit_hyperparams: true
  fit_interval: 10
  max_opt_iter: 100
  opt_bounds:
    length_scale: [0.5, 12.0]
    sigma_f: [0.1, 5.0]
    noise_level: [0.001, 1.0]
  opt_prior:
    length_scale_log_std: 0.5
  sklearn:
    normalize_y: true
    n_restarts_optimizer: 2
    alpha: 1e-10
    length_scale_bounds: [1e-2, 1e3]
    sigma_f_bounds: [1e-3, 1e3]
    noise_level_bounds: [1e-5, 1e-1]
```

## Configuration

HEDAC simulations are configured via YAML files. See `configs/default_params.yaml` for a complete example:

```yaml
simulation:
  num_steps: 2000              # Number of simulation steps
  num_agents: 1                # Number of agents
  dt: 0.1                      # Time step
  random_seed: 42              # Random seed for reproducibility

heat_equation:
  alpha: 0.1                   # Heat diffusion coefficient
  source_strength: 1.0         # Source term scaling
  beta: 0.01                   # Heat decay coefficient
  local_cooling: 0.1           # Local cooling coefficient

agents:
  max_velocity: 3.0            # Maximum velocity
  max_acceleration: 0.5        # Maximum acceleration
  agent_radius: 0.5            # Agent radius for coverage
```

See `YAML_CONFIG_GUIDE.md` for detailed configuration options.

## Folder Structure

```
hedac/
├── src/
│   ├── core/
│   │   ├── base.py              # Base classes and map loader
│   │   ├── hedac.py             # Main HEDAC algorithm
│   │   └── hedac_decentralized.py  # Decentralized version
│   ├── models/
│   │   └── agents.py            # Agent models
│   └── utils/
│       ├── math_utils.py        # Math utilities
│       └── visualize_gp.py      # GP debug visualization
├── examples/
│   ├── run_hedac.py             # Main simulation script
│   ├── run_gp_single_robot.py   # GP single-robot debug example
│   ├── run_decentralized_hedac.py  # Decentralized version
│   └── debug_gradient.py        # Debug visualization
├── configs/                      # Configuration files
├── maps/                         # Map files
├── pyproject.toml               # Project dependencies
└── README.md                    # This file
```

## Key Features

1. **Centralized GP Posterior**: Fixed-hyperparameter GP with mean/std grid predictions
2. **Uncertainty-Aware Goal Density**: Combines mean and std for exploration/exploitation
3. **Optional Dataset Filtering**: Focuses samples on uncertain areas
4. **GP Debug Visualization**: 4-panel view for step-by-step inspection
5. **Flexible Map Loading**: Supports obstacle-free and custom maps
6. **Configuration-based**: YAML configs for easy experimentation

## References

- Original Paper:  [HEDAC: Heat Equation Driven Area Coverage](https://www.sciencedirect.com/science/article/pii/S0952197622004316)

> Ivić, Stefan, Ante Sikirica, and Bojan Crnković. "Constrained multi-agent ergodic area surveying control based on finite element approximation of the potential field." Engineering applications of artificial intelligence 116 (2022): 105441.
