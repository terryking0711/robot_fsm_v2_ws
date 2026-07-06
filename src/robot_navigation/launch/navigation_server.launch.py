# ros2 launch robot_navigation navigation_server.launch.py
#
# 對齊 tdk_slam_ws real-robot 設定：
#   * Nav2 action:   /navigate_to_pose（tdk_nav2_manager nav_launch.py 的 bt_navigator）
#   * global_frame:  map（Cartographer pure localization 提供 map->odom）
#   * use_sim_time:  false（實車）

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
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'named_poses_file',
            default_value=default_named_poses_file,
            description='Path to named_poses.yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        Node(
            package='robot_navigation',
            executable='navigation_server',
            name='navigation_server',
            output='screen',
            parameters=[{
                'named_poses_file': named_poses_file,
                'nav2_action_name': '/navigate_to_pose',
                'global_frame': 'map',
                'server_wait_sec': 10.0,
                'default_timeout_sec': 60.0,
                'use_sim_time': use_sim_time,
            }]
        )
    ])