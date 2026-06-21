# ros2 launch robot_navigation named_pose_recorder.launch.py \
# named_poses_file:=/home/tdk/robot_fsm_v2_ws/src/robot_navigation/config/named_poses.yaml

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')

    named_poses_file = os.path.join(
        pkg_share,
        'config',
        'named_poses.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='named_pose_recorder',
            name='named_pose_recorder',
            output='screen',
            parameters=[{
                'named_poses_file': named_poses_file,
                'global_frame': 'map',
                'base_frame': 'base_footprint',
                'lookup_timeout_sec': 1.0,
                'decimal_places': 4,
                'allowed_pose_names': [
                    'leave_start_zone',
                    'stage1_entry',
                    'stage2_entry',
                ],
            }]
        )
    ])