"""Deterministic command-filter rollouts on the configured SITL truth field."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from docker.deployment_obstacle_filter import ObstacleFilter
from docker.deployment_ground_planner import ground_proposals
from src.core.deployment_scenario import build_scenario


def main():
    m = yaml.safe_load(Path('docker/scenarios/sitl_three_drone_two_obstacles.yaml').read_text())
    scenario = build_scenario(m)
    y, x = np.indices(scenario.high_field.shape)
    points = np.column_stack((x.ravel()-10., y.ravel()-10.))
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.imshow(scenario.high_field, origin='lower', extent=[-10,50,-10,50], cmap='viridis')
    minimum = float('inf')
    changed = 0
    for filtered in (False, True):
        p = np.array([[38., 22.], [20., 36.]])
        heading = np.zeros(2)
        traces = [p.copy()]
        f = ObstacleFilter(m, 'ground')
        for _ in range(1200):
            proposal = ground_proposals(points, scenario.high_field.ravel(), np.ones(len(points)), p, heading, max_speed=.5, gain=1.)
            velocities = proposal.velocities.copy()
            for i in range(2):
                if filtered:
                    velocities[i] = f.apply(p[i], velocities[i], .5, [[-9,49],[-9,49]], 2.5)
                    changed += int(np.linalg.norm(velocities[i]-proposal.velocities[i]) > 1e-9)
                    segment = .05 * velocities[i]
                    for center, radius in zip(f.centers, f.radii):
                        delta = p[i] - center
                        t = np.clip(-delta @ segment / max(segment @ segment, 1e-30), 0, 1)
                        minimum = min(minimum, float(np.linalg.norm(delta+t*segment)-radius))
            p += .05 * velocities
            heading = proposal.headings
            traces.append(p.copy())
        traces = np.array(traces)
        for i, color in enumerate(('cyan', 'orange')):
            ax.plot(*traces[:,i,:].T, color=color, linestyle='-' if filtered else '--', label=f'Ground {i+1}, '+('CBF' if filtered else 'nominal'))
    for circle in m['map']['obstacles']['circles']:
        ax.add_patch(plt.Circle(circle['center_m'], circle['radius_m'], color='gray'))
        ax.add_patch(plt.Circle(circle['center_m'], circle['radius_m']+1, fill=False, color='white', linestyle=':'))
    ax.set(xlabel='North [m]', ylabel='East [m]', title='SITL field: Lloyd proposals with command-rate CBF')
    ax.legend(fontsize=8)
    root = Path('docs/assets/sitl_obstacle_cbf'); root.mkdir(parents=True, exist_ok=True)
    fig.savefig(root/'summary.png', dpi=150, bbox_inches='tight'); plt.close(fig)
    metrics = dict(minimum_inflated_segment_clearance_m=minimum, changed_commands=changed, duration_s=60, dt_s=.05, note='Ideal velocity tracking, static truth-weighted Lloyd target; not a SITL flight')
    (root/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    print(json.dumps(metrics))
    assert minimum >= -1e-8 and changed > 0

if __name__ == '__main__':
    main()
