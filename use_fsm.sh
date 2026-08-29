#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "usage: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$workspace_dir/install/local_setup.bash" ]]; then
  echo "error: robot_fsm is not built; run $workspace_dir/build_fsm.sh first" >&2
  return 1
fi

# Reset ROS overlay search paths first.  Merely sourcing another setup file
# does not reorder a prefix that was already present, so an older
# /workspaces/install package could otherwise remain ahead of this workspace.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/local_setup.bash"

resolved_prefix="$(ros2 pkg prefix robot_fsm 2>/dev/null)"
if [[ "$resolved_prefix" != "$workspace_dir/install/robot_fsm" ]]; then
  echo "error: unexpected robot_fsm prefix: $resolved_prefix" >&2
  return 1
fi

echo "Using robot_fsm from $resolved_prefix"
unset workspace_dir resolved_prefix
