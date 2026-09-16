"""Lets the web control panel start robot_fsm_main.

rosbridge only gives a browser pub/sub + service-call access to the ROS
graph -- it cannot spawn a new process. robot_fsm_main is a one-shot
executable (started via `ros2 launch robot_fsm robot_bringup.launch.py`),
not something already running that the panel could just publish to. This
node bridges that gap: it stays up permanently and exposes a Trigger
service that does the actual `ros2 launch` on request.
"""
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

START_ROBOT_FSM_MAIN_SERVICE = '/fsm/start_robot_fsm_main'
LAUNCH_COMMAND = ['ros2', 'launch', 'robot_fsm', 'robot_bringup.launch.py']


class FsmLauncher(Node):
    def __init__(self):
        super().__init__('fsm_launcher')
        self._process = None
        self._lock = threading.Lock()
        self.create_service(Trigger, START_ROBOT_FSM_MAIN_SERVICE, self._on_start)
        self.get_logger().info(f'Ready: call {START_ROBOT_FSM_MAIN_SERVICE} to launch robot_fsm_main')

    def _on_start(self, request, response):
        with self._lock:
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
                    LAUNCH_COMMAND,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError as exc:
                response.success = False
                response.message = f'failed to launch robot_fsm_main: {exc}'
                return response

            response.success = True
            response.message = 'robot_fsm_main launch started'
            return response


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
