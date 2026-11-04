#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/mfgp_ros/install/setup.bash
set -u
exec python3 -m "$@"
