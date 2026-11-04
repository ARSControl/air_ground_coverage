"""Host deployment scripts select roles and never enable motion implicitly."""
import argparse
from pathlib import Path
import pytest
from docker import deploy


def config(tmp_path, role='drone'):
    scenario = tmp_path/'scenario.yaml'
    scenario.write_text('scenario fixture')
    interfaces = tmp_path/'mullet interfaces'
    interfaces.mkdir()
    (interfaces/'package.xml').write_text('<package/>')
    return dict(role=role, vehicle_id='uav_1' if role == 'drone' else None,
                scenario=str(scenario), scenario_sha256=deploy.digest(scenario),
                interfaces=str(interfaces), domain=42, image='mfgp-deployment:hardware',
                base_image='ros:jazzy-ros-base-noble')


def test_role_selection_and_motion_is_explicit(tmp_path):
    c = config(tmp_path)
    normal = deploy.commands(c, 'up')
    assert normal[-1][-2:] == ['-d', 'agent']
    assert deploy.environment(c, False)['ENABLE_COMMANDS'] == '0'
    assert deploy.environment(c, True)['ENABLE_COMMANDS'] == '1'
    c['role'] = 'laptop'
    c['vehicle_id'] = None
    assert deploy.commands(c, 'up')[-1][-2:] == ['central', 'frame-registry']
    with pytest.raises(ValueError, match='drone'):
        deploy.commands(c, 'up', motion=True)
    assert 'agent-uav1' not in deploy.commands(c, 'up')[-1]


def test_build_native_hardware_and_safe_argument_boundaries(tmp_path):
    c = config(tmp_path)
    command = deploy.commands(c, 'build')[0]
    assert command[:4] == ['docker', 'buildx', 'build', '--load']
    assert 'mullet_interfaces='+c['interfaces'] in command
    assert any(x.endswith('Dockerfile.hardware') for x in command)
    assert '--platform' not in command
    c['interfaces'] = str(tmp_path/'missing-package')
    with pytest.raises(ValueError, match='interfaces'):
        deploy.commands(c, 'build')


def test_manifest_changes_detected_and_role_tools_restricted(tmp_path):
    c = config(tmp_path)
    with pytest.raises(ValueError, match='laptop'):
        deploy.commands(c, 'check')
    Path(c['scenario']).write_text('changed')
    with pytest.raises(ValueError, match='changed'):
        deploy.commands(c, 'up')
    # Stopping and logs must remain available even if the manifest was edited.
    assert deploy.commands(c, 'stop')[-1][-2:] == ['stop', 'agent']


def test_tools_forward_arguments_and_ignore_ambient_motion(tmp_path, monkeypatch):
    c = config(tmp_path, 'laptop')
    monkeypatch.setenv('ENABLE_COMMANDS', '1')
    monkeypatch.setenv('MFGP_SCENARIO_FILE', '/wrong.yaml')
    env = deploy.environment(c, False)
    assert env['ENABLE_COMMANDS'] == '0'
    assert env['MFGP_SCENARIO_FILE'] == c['scenario']
    cmd = deploy.commands(c, 'mission', extra=['--duration','60','--output','/artifacts/trial.json'])[-1]
    assert cmd[-5:] == ['mission','--duration','60','--output','/artifacts/trial.json']


def test_init_persists_absolute_paths_and_refuses_overwrite(tmp_path):
    scenario = tmp_path/'scenario.yaml'; scenario.write_text('fixture')
    args = argparse.Namespace(role='drone', vehicle='uav_1', scenario=scenario,
                              interfaces=None, domain=42, image='test:local',
                              base_image='ros:jazzy-ros-base-noble', force=False)
    output = tmp_path/'config.json'
    deploy.initialize(args, output)
    assert output.exists()
    with pytest.raises(ValueError, match='exists'):
        deploy.initialize(args, output)
    args.vehicle = None
    with pytest.raises(ValueError, match='vehicle'):
        deploy.initialize(args, tmp_path/'other.json')


def test_actual_cli_uses_saved_config_and_stops_after_failed_validation(tmp_path, monkeypatch):
    import json
    import os
    import subprocess
    import sys
    c = config(tmp_path)
    saved = tmp_path/'host.json'; saved.write_text(json.dumps(c))
    log = tmp_path/'docker.log'
    stub = tmp_path/'docker'
    stub.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['DEPLOY_TEST_LOG'], 'a') as stream:
    stream.write(json.dumps(dict(args=sys.argv[1:], motion=os.environ.get('ENABLE_COMMANDS'), scenario=os.environ.get('MFGP_SCENARIO_FILE')))+'\\n')
if 'docker.validate_deployment' in sys.argv and os.environ.get('FAIL_VALIDATION'):
    raise SystemExit(7)
''')
    stub.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'], DEPLOY_TEST_LOG=str(log), ENABLE_COMMANDS='1')
    cmd = [sys.executable, str(deploy.ROOT/'docker/deploy.py'), '--config',str(saved),'up']
    subprocess.run(cmd, env=env, check=True, capture_output=True)
    rows = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(rows) == 2
    assert all(row['motion'] == '0' for row in rows)
    assert rows[-1]['args'][-1] == 'agent'
    assert '/bin/bash' in rows[0]['args']
    env['FAIL_VALIDATION'] = '1'
    result = subprocess.run(cmd, env=env, capture_output=True)
    assert result.returncode == 7
    assert len(log.read_text().splitlines()) == 3  # No up after failed validation.


def test_manifest_validator_vehicle_selection_and_cbf_config():
    import yaml
    from docker.validate_deployment import validate
    m = yaml.safe_load((deploy.ROOT/'docker/scenarios/sitl_three_drone_two_obstacles.yaml').read_text())
    assert validate(m, 'drone', 'uav_2')['vehicle_id'] == 'uav_2'
    with pytest.raises(ValueError, match='vehicle'):
        validate(m, 'drone', 'missing')
    assert validate(m, 'laptop')['scenario_hash']
    m['control']['obstacle_cbf']['alpha_per_s'] = -1
    with pytest.raises(ValueError, match='CBF'):
        validate(m, 'laptop')


def test_default_build_uses_locked_remote_commit(tmp_path):
    c = config(tmp_path)
    c['interfaces'] = None
    pin = deploy.interfaces_lock()
    command = deploy.commands(c, 'build')[0]
    assert 'mullet_interfaces='+pin['repository']+'#'+pin['commit'] in command
    assert re_full_commit(pin['commit'])
    c['interfaces_repository'] = pin['repository']
    c['interfaces_commit'] = 'main'
    with pytest.raises(ValueError, match='40'):
        deploy.commands(c, 'build')


def re_full_commit(value):
    import re
    return re.fullmatch(r'[0-9a-f]{40}', value)


def test_init_freezes_interface_pin(tmp_path):
    import json
    scenario = tmp_path/'scenario.yaml'; scenario.write_text('fixture')
    args = argparse.Namespace(role='laptop', vehicle=None, scenario=scenario,
                              interfaces=None, domain=42, image='test:local',
                              base_image='ros:jazzy-ros-base-noble', force=False)
    saved = tmp_path/'host.json'
    deploy.initialize(args, saved)
    data = json.loads(saved.read_text())
    assert data['interfaces_commit'] == deploy.interfaces_lock()['commit']
    assert data['interfaces_repository'] == deploy.interfaces_lock()['repository']
