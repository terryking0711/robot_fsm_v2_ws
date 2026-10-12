"""Lets the web control panel start robot_fsm_main.

rosbridge only gives a browser pub/sub + service-call access to the ROS
graph -- it cannot spawn a new process. robot_fsm_main is a one-shot
executable (started via `ros2 launch robot_fsm robot_bringup.launch.py`),
not something already running that the panel could just publish to. This
node bridges that gap: it stays up permanently and exposes one Trigger
service per competition field side that does the actual `ros2 launch` on
request (START NORMAL / START MIRROR / START DIT buttons in the panel).

The map itself is already loaded before this, by whichever desktop icon
started the stack ("TDK Loc_Normal" / "TDK Loc_Mirror" / "TDK Loc_DIT" ->
start.sh <side>),
which also passes that side in as the loaded_field_side parameter. A START
for the other side is refused, since the FSM's named poses would not match
the map cartographer/Nav2 are localizing on. Leave loaded_field_side empty
(e.g. start_mapping.sh) to skip that check.

/fsm/stop (STOP button) brings the robot to a halt without restarting the
container: zero /cmd_vel, SIGINT robot_fsm_main (and the navigation_server
launched with it), cancel every Nav2 goal -- Nav2 keeps driving a goal it
already accepted even after its client is gone -- then keep publishing zero
/cmd_vel for a moment so the last velocity the STM forwards is 0.
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

# dit = our own lab field (field_config.yaml sides.dit, skip_stage2 there).
FIELD_SIDES = ('normal', 'mirror', 'dit')
START_SERVICE_PREFIX = '/fsm/start_'  # -> /fsm/start_normal, /fsm/start_mirror, /fsm/start_dit
LOC_DESKTOP_ICONS = {'normal': 'TDK Loc_Normal', 'mirror': 'TDK Loc_Mirror', 'dit': 'TDK Loc_DIT'}
STOP_SERVICE = '/fsm/stop'

# A CancelGoal request with a zero goal id and zero stamp cancels ALL goals.
NAV2_ACTIONS = ('/navigate_to_pose', '/navigate_through_poses')

# pkill -f patterns: the `ros2 launch robot_bringup.launch.py` process and the
# robot_fsm_main binary itself (install/robot_fsm/lib/robot_fsm/robot_fsm_main),
# so a robot_fsm_main started from a terminal in this container is stopped too.
FSM_PROCESS_PATTERNS = ('robot_bringup.launch.py', 'lib/robot_fsm/robot_fsm_main')
FSM_BINARY_PATTERN = 'lib/robot_fsm/robot_fsm_main'
FSM_EXIT_TIMEOUT_SEC = 5.0

ZERO_CMD_VEL_RATE_HZ = 20.0
ZERO_CMD_VEL_BURST_SEC = 1.5


def launch_command(field_side):
    return ['ros2', 'launch', 'robot_fsm', 'robot_bringup.launch.py', f'field_side:={field_side}']


class FsmLauncher(Node):
    def __init__(self):
        super().__init__('fsm_launcher')
        self._process = None
        self._lock = threading.Lock()
        self._loaded_field_side = self.declare_parameter('loaded_field_side', '').value
        for field_side in FIELD_SIDES:
            self.create_service(
                Trigger,
                START_SERVICE_PREFIX + field_side,
                lambda request, response, side=field_side: self._on_start(side, response))

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav2_cancel_clients = {
            action: self.create_client(CancelGoal, f'{action}/_action/cancel_goal')
            for action in NAV2_ACTIONS
        }
        self.create_service(Trigger, STOP_SERVICE, lambda request, response: self._on_stop(response))
        self.get_logger().info(
            f'Ready: call {START_SERVICE_PREFIX}{{{",".join(FIELD_SIDES)}}} to launch robot_fsm_main, '
            f'{STOP_SERVICE} to stop it (loaded map side: {self._loaded_field_side or "unchecked"})')

    def _on_start(self, field_side, response):
        with self._lock:
            if self._loaded_field_side and field_side != self._loaded_field_side:
                response.success = False
                response.message = (
                    f'map loaded is {self._loaded_field_side.upper()}, not {field_side.upper()} -- '
                    f'use the {LOC_DESKTOP_ICONS[field_side]} desktop icon first')
                self.get_logger().error(response.message)
                return response

            # Only catches double-launches issued through this same node --
            # if robot_fsm_main was started some other way (a terminal, the
            # future physical switch) this won't see it. Good enough for now;
            # revisit with a `ros2 node list` check if that turns out to matter.
            if self._process is not None and self._process.poll() is None:
                response.success = False
                response.message = 'robot_fsm_main launch already in progress'
                return response

            try:
                self._process = subprocess.Popen(
                    launch_command(field_side),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError as exc:
                response.success = False
                response.message = f'failed to launch robot_fsm_main: {exc}'
                return response

            response.success = True
            response.message = f'robot_fsm_main launch started ({field_side.upper()})'
            self.get_logger().info(response.message)
            return response

    def _on_stop(self, response):
        with self._lock:
            self._publish_zero_cmd_vel(0.0)
            was_running = self._signal_fsm(signal.SIGINT)
            canceled = self._cancel_nav2_goals()

            exited = self._wait_fsm_exit(FSM_EXIT_TIMEOUT_SEC)
            if not exited:
                self.get_logger().warn('robot_fsm_main ignored SIGINT, sending SIGKILL')
                self._signal_fsm(signal.SIGKILL)
                exited = self._wait_fsm_exit(1.0)

            self._publish_zero_cmd_vel(ZERO_CMD_VEL_BURST_SEC)

            if not exited:
                response.success = False
                response.message = 'robot_fsm_main is still running (cmd_vel zeroed)'
            else:
                response.success = True
                response.message = (
                    ('STOPPED robot_fsm_main' if was_running else 'robot_fsm_main was not running')
                    + (', Nav2 goals canceled' if canceled else ', Nav2 not reachable')
                    + ', cmd_vel = 0')
            self.get_logger().info(response.message)
            return response

    def _publish_zero_cmd_vel(self, duration_sec):
        """Publish zero /cmd_vel once, then keep repeating it for duration_sec."""
        end = time.monotonic() + duration_sec
        while True:
            self._cmd_vel_pub.publish(Twist())
            if time.monotonic() >= end:
                return
            time.sleep(1.0 / ZERO_CMD_VEL_RATE_HZ)

    def _signal_fsm(self, sig):
        """Signal robot_fsm_main's launch; returns whether any such process was running."""
        was_running = self._fsm_running()
        if self._process is not None and self._process.poll() is None:
            try:
                os.killpg(self._process.pid, sig)  # start_new_session -> pgid == pid
            except ProcessLookupError:
                pass
        for pattern in FSM_PROCESS_PATTERNS:
            subprocess.run(['pkill', f'-{sig.value}', '-f', pattern], check=False)
        return was_running

    def _wait_fsm_exit(self, timeout_sec):
        end = time.monotonic() + timeout_sec
        while self._fsm_running():
            if time.monotonic() >= end:
                return False
            time.sleep(0.1)
        if self._process is not None:
            try:
                self._process.wait(timeout=1.0)  # reap the ros2 launch parent
            except subprocess.TimeoutExpired:
                pass
        return True

    def _fsm_running(self):
        return subprocess.run(
            ['pgrep', '-f', FSM_BINARY_PATTERN], stdout=subprocess.DEVNULL, check=False
        ).returncode == 0

    def _cancel_nav2_goals(self):
        """Fire-and-forget cancel of every Nav2 goal; returns whether any cancel was sent."""
        sent = False
        for action, client in self._nav2_cancel_clients.items():
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
                sent = True
            else:
                self.get_logger().warn(f'{action} cancel service not available')
        return sent


def main():
    rclpy.init()
    node = FsmLauncher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
