from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


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
            executable='navigation_server',
            name='navigation_server',
            output='screen',
            parameters=[{
                'named_poses_file': named_poses_file,
                'nav2_action_name': '/navigate_to_pose',
                'global_frame': 'map'
            }]
        )
    ])