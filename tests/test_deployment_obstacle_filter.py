from pathlib import Path
import numpy as np
import yaml
from docker.deployment_obstacle_filter import ObstacleFilter
from docker.deployment_command_gate import accepted_velocity
from src.core.deployment_scenario import build_scenario


def manifest():
    return yaml.safe_load(Path('docker/scenarios/sitl_three_drone_two_obstacles.yaml').read_text())


def test_field_unchanged_and_two_circles():
    m = manifest()
    old = yaml.safe_load(Path('docker/scenarios/sitl_three_drone_1000_samples_high_information_no_replacement.yaml').read_text())
    assert len(m['map']['obstacles']['circles']) == 2
    assert m['field'] == old['field']
    np.testing.assert_array_equal(build_scenario(m).high_field, build_scenario(old).high_field)
    assert m['estimator'] == old['estimator']


def test_actual_gate_filters_ground_and_holds_inside():
    f = ObstacleFilter(manifest(), 'ground')
    assert f.last_status == 'awaiting_command'
    plan = dict(scenario_hash='s', vehicle_id='u', timestamp=10, expires_at=12,
                velocity=[-.5, 0], heading=0)
    args = dict(registered=True, mission=True, max_speed=.5, bounds=[[-10, 50], [-10, 50]], obstacle_filter=f)
    result = accepted_velocity(plan, 's', 'u', 10.1, position=[29.1, 25], **args)
    assert -.1 < result[0] < 0
    assert f.last_status == 'filtered'
    assert accepted_velocity(plan, 's', 'u', 10.1, position=[25, 25], **args)[:2] == (0, 0)
    assert f.last_status == 'unsafe_or_infeasible_hold'
    args['mission'] = False
    assert accepted_velocity(plan, 's', 'u', 10.1, position=[29.1, 25], **args) is None


def test_aerial_and_legacy_unchanged():
    m = manifest()
    assert ObstacleFilter(m, 'aerial').apply([25,25], [.5,0], .5, None, 2.5) == [.5,0]
    del m['control']['obstacle_cbf']
    assert not ObstacleFilter(m, 'ground').enabled
