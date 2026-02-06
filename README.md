# HEDAC: Heat Equation Driven Area Coverage

A cleaned and improved implementation of the HEDAC algorithm for multi-agent
ergodic control based on the paper by Sarah Dean et al.

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
```

### Python API Usage

```python
import numpy as np
from src.core.base import HEDACParams, MapLoader
from src.core.hedac import HEDACAlgorithm
from src.models.agents import DoubleIntegratorAgent, AgentTeam

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
│       └── math_utils.py        # Math utilities
├── examples/
│   ├── run_hedac.py             # Main simulation script
│   ├── run_decentralized_hedac.py  # Decentralized version
│   └── debug_gradient.py        # Debug visualization
├── configs/                      # Configuration files
├── maps/                         # Map files
├── pyproject.toml               # Project dependencies
└── README.md                    # This file
```

## Key Features

1. **Correct Ergodic Metric**: Computes L2 distance between normalized distributions
2. **Flexible Map Loading**: Supports obstacle-free and custom maps (TBA)
3. **Configuration-based**: YAML configs for easy experimentation

## References

- Original Paper:  [HEDAC: Heat Equation Driven Area Coverage](https://www.sciencedirect.com/science/article/pii/S0952197622004316)

> Ivić, Stefan, Ante Sikirica, and Bojan Crnković. "Constrained multi-agent ergodic area surveying control based on finite element approximation of the potential field." Engineering applications of artificial intelligence 116 (2022): 105441.
