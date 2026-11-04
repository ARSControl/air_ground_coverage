"""Offline manifest and runtime compatibility check inside the built image."""
import argparse
import json
from pathlib import Path
import platform
import xml.etree.ElementTree as ET
import yaml

from docker.deployment_obstacle_filter import ObstacleFilter
from docker.deployment_settings import DeploymentSettings, scenario_path
from src.core.deployment_scenario import build_scenario


def validate(manifest, role, vehicle_id=None):
    scenario = build_scenario(manifest)
    settings = DeploymentSettings.from_manifest(manifest)
    vehicles = manifest['vehicles']
    if sum(v['role'] == 'aerial' for v in vehicles) != 1:
        raise ValueError('current deployment requires exactly one aerial vehicle')
    for v in vehicles:
        ObstacleFilter(manifest, v['role'])
    if role == 'drone':
        matches = [v for v in vehicles if v['id'] == vehicle_id]
        if len(matches) != 1:
            raise ValueError('vehicle is not uniquely present in scenario')
    elif role != 'laptop':
        raise ValueError('unknown host role')
    return dict(scenario_id=manifest['scenario_id'], scenario_hash=scenario.scenario_hash,
                role=role, vehicle_id=vehicle_id, architecture=platform.machine(),
                configured_command_hz=settings.command_hz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['drone','laptop'], required=True)
    parser.add_argument('--vehicle')
    args = parser.parse_args()
    # Import ROS wrappers without creating nodes or publishing anything.
    from mullet_interfaces.msg import DroneHeartbeat, MissionCommand
    from docker.ros_agent import SensingAgent
    from docker.ros_central import CentralNode
    assert DroneHeartbeat is not None and MissionCommand is not None
    assert SensingAgent is not None and CentralNode is not None
    manifest = yaml.safe_load(Path(scenario_path()).read_text())
    result = validate(manifest, args.role, args.vehicle)
    package = Path('/opt/mfgp_ros/src/mullet_interfaces/package.xml')
    result['mullet_interfaces_version'] = ET.parse(package).getroot().findtext('version')
    print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
