"""Exercise the actual shell wrapper using a deterministic Docker stub."""
import os
from pathlib import Path
import subprocess


def test_check_reuses_running_scenario_and_keeps_explicit_override(tmp_path):
    stub = tmp_path / 'docker'
    stub.write_text('''#!/bin/bash
if [[ "$*" == *"ps -q central"* ]]; then
  echo central-id
elif [[ "$1" == "inspect" ]]; then
  echo /tmp/running-two-obstacles.yaml
else
  echo "selected=$MFGP_SCENARIO_FILE args=$*"
fi
''')
    stub.chmod(0o755)
    env = dict(os.environ, PATH=f'{tmp_path}:'+os.environ['PATH'])
    env.pop('MFGP_SCENARIO_FILE', None)
    result = subprocess.run(['bash', 'docker/run_sitl.sh', 'check', '--timeout', '8'], env=env, capture_output=True, text=True, check=True)
    assert 'selected=/tmp/running-two-obstacles.yaml' in result.stdout
    assert 'readiness --timeout 8' in result.stdout
    env['MFGP_SCENARIO_FILE'] = '/tmp/explicit.yaml'
    result = subprocess.run(['bash', 'docker/run_sitl.sh', 'check'], env=env, capture_output=True, text=True, check=True)
    assert 'selected=/tmp/explicit.yaml' in result.stdout
