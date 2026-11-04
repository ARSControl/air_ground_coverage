"""Deployment command-plan visual from real CLI functions and validation outputs."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from docker.deploy import ROOT, commands, digest, interfaces_lock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tests', type=Path, required=True)
    parser.add_argument('--smoke', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT/'docs/assets/hardware_deployment/summary.png')
    args = parser.parse_args()
    suite = ET.parse(args.tests).getroot().find('testsuite')
    smoke = json.loads(args.smoke.read_text())
    scenario = ROOT/'docker/scenarios/sitl_three_drone_two_obstacles.yaml'
    c = dict(role='drone', vehicle_id='uav_1', scenario=str(scenario), scenario_sha256=digest(scenario), image='mfgp-deployment:hardware', domain=42)
    drone = commands(c, 'up')[-1]
    c.update(role='laptop', vehicle_id=None)
    laptop = commands(c, 'up')[-1]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.set_axis_off()
    fig.suptitle('Pinned MULLET interfaces: '+interfaces_lock()['commit'], fontsize=10)
    for x, title, text in (
        (.02, 'Each onboard computer', 'Existing MULLET container\n+ MFGP '+drone[-1]+' container\nLocal planner + obstacle CBF\nCommand output disabled by default'),
        (.55, 'Laptop', 'Only '+' + '.join(laptop[-2:])+'\nOne shared field and GP\nReadiness + mission recorder\nNo duplicate drone agents')):
        ax.text(x, .88, title, transform=ax.transAxes, fontsize=14, weight='bold')
        ax.text(x, .72, text, transform=ax.transAxes, va='top', fontsize=11,
                bbox=dict(boxstyle='round,pad=.8', facecolor='#eef4fb', edgecolor='#456'))
    passed = int(suite.attrib['tests']) - int(suite.attrib['failures']) - int(suite.attrib['errors']) - int(suite.attrib.get('skipped', 0))
    ax.text(.02,.14, f"Measured validation: {passed} tests passed; offline hardware-image GP smoke\nLOW={smoke['low_count']}, HIGH={smoke['high_count']}, aerial target change={smoke['target_max_abs_change']:.6g}", transform=ax.transAxes, fontsize=11)
    ax.text(.02,.01,'Native AMD64 image validation; ARM64 and physical flight are not validated here.', transform=ax.transAxes, fontsize=10)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output,dpi=150,bbox_inches='tight');plt.close(fig)
    args.output.with_suffix('.json').write_text(json.dumps(dict(passed_tests=passed, smoke=smoke, drone_services=drone[-1:], laptop_services=laptop[-2:]),indent=2)+'\n')


if __name__ == '__main__':
    main()
