# YAML Configuration System

The HEDAC implementation now uses YAML files for configuration instead of Python dataclasses. This provides better flexibility, readability, and ease of use.

## Quick Start

### 1. Create a Config File

```yaml
# my_config.yaml
simulation:
  num_steps: 1000
  num_agents: 4
  dt: 0.1
  random_seed: 42

heat_equation:
  alpha: 0.1
  source_strength: 1.0
  beta: 0.01
  local_cooling: 0.1

agents:
  max_velocity: 1.0
  max_acceleration: 0.5
  max_angular_velocity: 0.785
  max_angular_acceleration: 0.393
  dt_agent: 0.1
  agent_radius: 0.5
  min_kernel_val: 0.01

map:
  # Option 1: Create obstacle-free map
  size: [50, 50]
  resolution: 1.0
  
  # Option 2: Load from file (comment out size above)
  # path: "path/to/map.npy"

goal_density:
  type: "gaussian_mixture"
  num_peaks: 3

output:
  save_results: true
  results_path: "output/results.npz"
  verbose: true
  print_frequency: 100
```

### 2. Load and Run

```python
from hedac.src.core.base import HEDACParams
from hedac.examples.run_hedac import run_hedac_from_config

# Load parameters
params = HEDACParams.from_yaml('my_config.yaml')

# Run simulation
results, map_array, goal_density, agent_team = run_hedac_from_config('my_config.yaml')
```

### 3. Command Line Usage

```bash
# Run with default config
cd hedac
python3 examples/run_hedac.py

# Run with custom config
python3 examples/run_hedac.py --config configs/complex_map_example.yaml

# Run without plotting
python3 examples/run_hedac.py --config my_config.yaml --no-plot
```

## Configuration Schema

### `simulation` Section

```yaml
simulation:
  num_steps: 1000              # Number of simulation steps
  num_agents: 4                # Number of agents
  dt: 0.1                      # Time step for heat equation
  random_seed: 42              # Random seed for reproducibility
```

### `heat_equation` Section

```yaml
heat_equation:
  alpha: 0.1                   # Heat diffusion coefficient
  source_strength: 1.0         # Source term scaling
  beta: 0.01                   # Heat decay coefficient
  local_cooling: 0.1           # Local cooling coefficient
```

### `agents` Section

```yaml
agents:
  model_type: "double_integrator"  # Options: "double_integrator", "dubins"
  max_velocity: 1.0            # Maximum velocity (m/s)
  max_acceleration: 0.5        # Maximum acceleration (m/s^2)
  max_angular_velocity: 0.785  # Maximum angular velocity (rad/s)
  max_angular_acceleration: 0.393  # Maximum angular acceleration (rad/s^2)
  dt_agent: 0.1                # Agent time step
  agent_radius: 0.5            # Agent radius for coverage block
  min_kernel_val: 0.01         # Minimum kernel value threshold

  dubins:
    forward_speed: 5.0          # Constant forward speed (m/s)
    max_bank_angle: 30.0        # Maximum bank angle (deg)
```

### `sensor` Section

```yaml
sensor:
  fov_degrees: 90.0            # Field of view angle in degrees
  fov_depth: 5.0               # Field of view depth in meters
```

### `multi_agent` Section

```yaml
multi_agent:
  sensing_range: 10.0          # Range for agent communication
  min_safe_distance: 1.0       # Minimum safe distance between agents
```

### `map` Section

**Option 1: Obstacle-free map**
```yaml
map:
  size: [50, 50]               # [height, width] in cells
  resolution: 1.0              # Meters per cell
```

**Option 2: Load from file**
```yaml
map:
  path: "maps/simple_map.npy"  # Path to .npy file
  # Resolution will be auto-detected
```

**Note:** Map files should be numpy arrays with:
- `0` = free space
- `1` = obstacles

### `goal_density` Section

**Option 1: Gaussian mixture**
```yaml
goal_density:
  type: "gaussian_mixture"
  num_peaks: 3                 # Number of Gaussian peaks
```

**Option 2: Uniform**
```yaml
goal_density:
  type: "uniform"
```

**Option 3: Load from file**
```yaml
goal_density:
  type: "file"
  file_path: "goal_densities/custom.npy"
```

### `visualization` Section

```yaml
visualization:
  plot_frequency: 10           # Plot every N steps (0 = no plotting)
  save_video: false            # Save video of simulation
  video_fps: 30                # Video frames per second
  video_path: "output/hedac_simulation.mp4"
```

### `output` Section

```yaml
output:
  save_results: true           # Save results to file
  results_path: "output/results.npz"
  verbose: true                # Print progress
  print_frequency: 100         # Print every N steps
```

## Accessing Parameters in Code

### Direct Attribute Access

```python
params = HEDACParams.from_yaml('config.yaml')

# Access top-level params
print(params.num_agents)
print(params.num_steps)
print(params.alpha)
```

### Nested Key Access

```python
# Access nested params with dot notation
alpha = params.get('heat_equation.alpha')
max_vel = params.get('agents.max_velocity')
map_path = params.get('map.path')
```

### Dictionary-Style Access

```python
# Access any param
print(params['simulation.num_steps'])
print(params['heat_equation'])
```

### Default Values

```python
# Provide default if key doesn't exist
value = params.get('some.missing.key', default_value=42)
```

## Example Config Files

Three example configs are provided:

1. **`configs/default_params.yaml`** - Default configuration for general use
2. **`configs/complex_map_example.yaml`** - Example with existing map file
3. **`configs/single_agent_simple.yaml`** - Minimal single-agent config

## Migrating from Old Dataclass

**Old way (dataclass):**
```python
from hedac.src.core.base import HEDACParams

params = HEDACParams(
    nb_data_points=1000,
    nb_agents=4,
    dt=0.1,
    alpha=0.1,
    ...
)
```

**New way (YAML):**
```python
params = HEDACParams.from_yaml('config.yaml')
```

Or create from dict:
```python
config_dict = {
    'simulation': {'num_steps': 1000, 'num_agents': 4, ...},
    'heat_equation': {'alpha': 0.1, ...},
    ...
}
params = HEDACParams.from_dict(config_dict)
```

## Benefits

1. **No code changes needed** to modify parameters
2. **Human-readable** configuration files
3. **Version control friendly** - easy to track parameter changes
4. **Multiple configs** - easily switch between different scenarios
5. **Nested structure** - organized parameter groups
6. **Comments** - document your parameter choices

## Advanced Usage

### Programmatic Config Creation

```python
import yaml

# Create config programmatically
config = {
    'simulation': {
        'num_steps': 500,
        'num_agents': 2
    },
    'map': {
        'size': [30, 30]
    }
}

# Save to file
with open('custom_config.yaml', 'w') as f:
    yaml.dump(config, f)

# Load and use
params = HEDACParams.from_yaml('custom_config.yaml')
```

### Config Validation

```python
# Check if required keys exist
if params.get('map.path') is None and params.get('map.size') is None:
    raise ValueError("Must specify either map.path or map.size")

# Validate parameter ranges
if params.alpha < 0 or params.alpha > 1:
    raise ValueError("alpha must be between 0 and 1")
```

## Troubleshooting

### ImportError: No module named 'yaml'

Install PyYAML:
```bash
pip install pyyaml
```

### Config file not found

Check the path is correct relative to the working directory:
```python
import os
config_path = os.path.join(os.path.dirname(__file__), 'configs', 'default_params.yaml')
params = HEDACParams.from_yaml(config_path)
```

### Missing required parameters

The system uses sensible defaults for all parameters. Only specify what you need to change from defaults.

## Dependencies

- Python 3.7+
- PyYAML (`pip install pyyaml`)
- NumPy
- Other standard HEDAC dependencies
