# ros2 launch robot_navigation navigation_server.launch.py
#
# 對齊 tdk_slam_ws real-robot 設定：
#   * Nav2 action:   /navigate_to_pose（tdk_nav2_manager nav_launch.py 的 bt_navigator）
#   * global_frame:  world（方案 A）
#       named_poses.yaml 全部以 world frame（場地左下角原點）表示，
#       goal 的 frame_id 會是 "world"；Nav2 Humble 的 planner_server 有
#       transformPosesToGlobalFrame，只要 world -> map 靜態 TF 存在
#       （由 tdk_slam_ws spawn_launch.py 的 world_to_map_static_publisher 發布），
#       就會自動把 goal 轉進 map frame。
#   * use_sim_time:  false（實車）

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FIELD_CONFIG_FILE = '/workspaces/field_config.yaml'


def generate_launch_description():
    # 單獨啟動時預設用 field_config.yaml default_side 的 named_poses；
    # 從 robot_bringup.launch.py 進來時會直接指定該場地的 named_poses_file。
    with open(FIELD_CONFIG_FILE) as f:
        field_config = yaml.safe_load(f)
    default_named_poses_file = \
        field_config['sides'][field_config['default_side']]['named_poses_file']

    named_poses_file = LaunchConfiguration('named_poses_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'named_poses_file',
            default_value=default_named_poses_file,
            description='Path to named_poses.yaml (world frame poses)'
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
                # 方案 A：goal 以 world frame 發出，Nav2 自行轉換
                'global_frame': 'world',
                'server_wait_sec': 10.0,
                'default_timeout_sec': 60.0,
                'use_sim_time': use_sim_time,
            }]
        )
    ])
