#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace_dir/use_fsm.sh"
exec ros2 launch robot_fsm robot_bringup.launch.py field_side:=dit stage3_motion_test:=true "$@"
