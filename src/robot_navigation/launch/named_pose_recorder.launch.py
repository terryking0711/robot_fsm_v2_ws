# ros2 launch robot_navigation named_pose_recorder.launch.py \
#   named_poses_file:=/home/tdk/robot_fsm_v2_ws/src/robot_navigation/config/named_poses.yaml
#
# 錄完新的點之後，讓 navigation_server 熱重載（不用重啟）：
#   ros2 service call /reload_named_poses std_srvs/srv/Trigger

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')

    default_named_poses_file = os.path.join(
        pkg_share,
        'config',
        'named_poses.yaml'
    )

    named_poses_file = LaunchConfiguration('named_poses_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'named_poses_file',
            default_value=default_named_poses_file,
            description='Path to named_poses.yaml to update'
        ),

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
                # 與 mission_controller / stage FSM 實際會用到的點對齊
                'allowed_pose_names': [
                    'leave_start_zone',
                    'stage1_entry',
                    'stage2_entry',
                    'stage3_entry',
                    'stage3_pick_pose',
                    'stage3_stack_pose',
                ],
            }]
        )
    ])