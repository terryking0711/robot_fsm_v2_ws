#!/usr/bin/env bash
set -euo pipefail

# Build from a clean ROS environment so an older robot_fsm can never become
# an underlay of the generated setup files.
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: run this inside the ROS Docker container" >&2
  exit 1
fi

echo "Building only from: $workspace_dir/src"
echo "Removing stale build/install/log outputs from this workspace..."
rm -rf -- "$workspace_dir/build" "$workspace_dir/install" "$workspace_dir/log"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
# ROS 2 Humble's generated setup scripts read a few optional variables without
# nounset guards.
set +u
source /opt/ros/humble/setup.bash
set -u

colcon --log-base "$workspace_dir/log" build \
  --base-paths "$workspace_dir/src" \
  --build-base "$workspace_dir/build" \
  --install-base "$workspace_dir/install" \
  --symlink-install \
  --event-handlers console_cohesion+

set +u
source "$workspace_dir/install/local_setup.bash"
set -u

resolved_prefix="$(ros2 pkg prefix robot_fsm)"
expected_prefix="$workspace_dir/install/robot_fsm"
if [[ "$resolved_prefix" != "$expected_prefix" ]]; then
  echo "error: robot_fsm resolved to '$resolved_prefix', expected '$expected_prefix'" >&2
  exit 1
fi

executable="$resolved_prefix/lib/robot_fsm/robot_fsm_main"
launch_file="$resolved_prefix/share/robot_fsm/launch/robot_bringup.launch.py"
[[ -x "$executable" ]] || { echo "error: missing executable: $executable" >&2; exit 1; }
[[ -f "$launch_file" ]] || { echo "error: missing launch file: $launch_file" >&2; exit 1; }

echo
echo "Build verified:"
echo "  package:    $resolved_prefix"
echo "  executable: $executable"
echo "  launch:     $launch_file"
echo
echo "In each new terminal run:"
echo "  source $workspace_dir/use_fsm.sh"
echo "Do not source $workspace_dir/install/setup.bash directly."
