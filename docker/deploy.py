"""Build and operate this repository's containers beside an existing MULLET stack.

Host dependency: Python 3 standard library, Docker Engine, Compose and Buildx.
Run `python3 docker/deploy.py --help`. No SSH or remote operations are performed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / 'docker/deployment.local.json'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def interfaces_lock():
    """Return the reviewed deployed revision, never a moving branch name."""
    return json.loads((ROOT/'docker/mullet_interfaces.lock.json').read_text())


def interface_context(config):
    local = config.get('interfaces')
    if local:
        if not (Path(local)/'package.xml').is_file():
            raise ValueError('local --interfaces path must contain package.xml')
        return local
    lock = interfaces_lock()
    repository = config.get('interfaces_repository', lock['repository'])
    commit = config.get('interfaces_commit', lock['commit'])
    if not isinstance(commit, str) or not re.fullmatch(r'[0-9a-f]{40}', commit):
        raise ValueError('interfaces commit must be a full 40-character lowercase Git SHA')
    if not isinstance(repository, str) or not repository.startswith('https://') or '#' in repository:
        raise ValueError('interfaces repository must be an HTTPS Git URL without a fragment')
    return repository+'#'+commit


def initialize(args, path):
    if args.role == 'drone' and (not args.vehicle or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', args.vehicle)):
        raise ValueError('drone requires a valid --vehicle scenario ID')
    if args.role == 'laptop' and args.vehicle:
        raise ValueError('laptop does not take --vehicle')
    if path.exists() and not args.force:
        raise ValueError(f'{path} exists; use init --force to replace it')
    if not 0 <= args.domain <= 101:
        raise ValueError('--domain must be in the portable DDS range 0..101')
    scenario = args.scenario.expanduser().resolve(strict=True)
    interfaces = args.interfaces.expanduser().resolve(strict=True) if args.interfaces else None
    if interfaces and not (interfaces/'package.xml').is_file():
        raise ValueError('--interfaces must point to the mullet_interfaces package directory')
    lock = interfaces_lock()
    data = dict(interfaces_repository=lock['repository'], interfaces_commit=lock['commit'],
                role=args.role, vehicle_id=args.vehicle, domain=args.domain,
                scenario=str(scenario), scenario_sha256=digest(scenario),
                interfaces=str(interfaces) if interfaces else None,
                image=args.image, base_image=args.base_image)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2)+'\n')
    print(f'Wrote {path}; role={args.role}, commands disabled by default')


def environment(config, motion=False):
    env = os.environ.copy()
    env.update(MFGP_SCENARIO_FILE=config['scenario'], MFGP_IMAGE=config['image'],
               ROS_DOMAIN_ID=str(config['domain']), ENABLE_COMMANDS='1' if motion else '0',
               VEHICLE_ID=config.get('vehicle_id') or '')
    return env


def commands(config, action, *, motion=False, extra=()):
    role = config['role']
    if role not in ('drone', 'laptop'):
        raise ValueError('invalid deployment role; run init again')
    if motion and (role != 'drone' or action != 'up'):
        raise ValueError('--motion is supported only for drone up')
    if action in ('check', 'mission') and role != 'laptop':
        raise ValueError(f'{action} must run on the laptop')
    if extra and action not in ('check', 'mission'):
        raise ValueError('extra options are supported only for check/mission')
    if action not in ('stop', 'logs', 'status'):
        if digest(config['scenario']) != config['scenario_sha256']:
            raise ValueError('scenario changed since init; archive the original and run init --force for an intentional change')
    compose_file = ROOT/'docker'/('compose.drone.yaml' if role == 'drone' else 'compose.yaml')
    compose = ['docker', 'compose', '-f', str(compose_file)]
    services = ['agent'] if role == 'drone' else ['central', 'frame-registry']
    # Separate project names avoid touching an existing SITL or MULLET stack.
    compose += ['-p', 'mfgp_onboard' if role == 'drone' else 'mfgp_laptop']
    if action == 'build':
        interfaces = interface_context(config)
        return [['docker','buildx','build','--load','--file',str(ROOT/'docker/Dockerfile.hardware'),
                 '--build-context','mullet_interfaces='+interfaces,
                 '--build-arg','BASE_IMAGE='+config['base_image'], '--tag',config['image'],str(ROOT)]]
    validate = compose+['run','--rm','-T','--no-deps','--entrypoint',
                        '/bin/bash', services[0], '/opt/mfgp/docker/entrypoint.sh',
                        'docker.validate_deployment', '--role', role]
    if role == 'drone':
        validate += ['--vehicle', config['vehicle_id']]
    if action == 'validate':
        return [validate]
    if action == 'up':
        return [validate, compose+['up','--no-build','-d',*services]]
    if action == 'smoke':
        return [['docker','run','--rm','--network','none',
                 '--mount',f'type=bind,source={config["scenario"]},target=/config/scenario.yaml,readonly',
                 config['image'],'docker.deployment_smoke','--scenario','/config/scenario.yaml']]
    if action in ('check', 'mission'):
        return [compose+['run','--rm','-T','--no-deps', 'readiness' if action == 'check' else 'mission', *extra]]
    if action == 'logs':
        return [compose+['logs','--tail','100','-f',*services]]
    if action == 'status':
        return [compose+['ps',*services]]
    if action == 'stop':
        return [compose+['stop',*services]]
    raise ValueError('unknown deployment action')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--dry-run', action='store_true', help='Print Docker commands without executing them')
    sub = parser.add_subparsers(dest='action', required=True)
    init = sub.add_parser('init', help='Save settings for this machine; no Docker operations')
    init.add_argument('--role', choices=['drone','laptop'], required=True)
    init.add_argument('--vehicle')
    init.add_argument('--scenario', type=Path, default=ROOT/'docker/scenarios/sitl_three_drone_two_obstacles.yaml')
    init.add_argument('--interfaces', type=Path, help='Optional local package override; default downloads the pinned deployed Git revision')
    init.add_argument('--domain', type=int, default=42)
    init.add_argument('--image', default='mfgp-deployment:hardware')
    init.add_argument('--base-image', default='ghcr.io/skymule-ai/mullet-base:v0.5.1')
    init.add_argument('--force', action='store_true')
    for action in ('build','validate','smoke','up','stop','status','logs','check','mission'):
        command = sub.add_parser(action)
        if action == 'up':
            command.add_argument('--motion', action='store_true')
    args, extra = parser.parse_known_args(argv)
    try:
        if args.action == 'init':
            if extra or args.dry_run:
                raise ValueError('init does not support extra arguments or --dry-run')
            initialize(args, args.config.expanduser().resolve())
            return 0
        config = json.loads(args.config.expanduser().read_text())
        motion = getattr(args, 'motion', False)
        plan = commands(config, args.action, motion=motion, extra=extra)
        env = environment(config, motion)
        print(f'Role={config["role"]}, domain={config["domain"]}, image={config["image"]}, ENABLE_COMMANDS={env["ENABLE_COMMANDS"]}', flush=True)
        for command in plan:
            print('+ '+shlex.join(command), flush=True)
            if not args.dry_run:
                subprocess.run(command, env=env, cwd=ROOT, check=True)
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f'Deployment failed: {exc}', file=sys.stderr)
        return exc.returncode if isinstance(exc, subprocess.CalledProcessError) else 1


if __name__ == '__main__':
    raise SystemExit(main())
