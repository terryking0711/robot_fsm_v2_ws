# 啟動 FSM 主程式，並依 enable_navigation 決定要不要一併啟動 navigation_server。
#
#   比賽 / 完整模式（預設，導航啟用）：
#     ros2 launch robot_fsm robot_bringup.launch.py
#
#   機構單獨測試模式（導航關閉）：
#     ros2 launch robot_fsm robot_bringup.launch.py enable_navigation:=false
#       -> 不啟動 navigation_server
#       -> robot_fsm_main 內所有導航步驟直接視為抵達、定位直接視為成功
#          （等同先前 mission_test branch 的行為）
#
# 注意：
#   1. enable_navigation:=true 只會啟動「本 workspace 的」navigation_server。
#      Nav2 / Cartographer / localization_manager 仍要另外從 tdk_slam_ws 啟動。
#   2. cmd_vel -> /mecanum/cmd_vel 的橋接（stm_communication_node）不在這裡，
#      請另外跑 nav_cmd_bridge.launch.py。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_navigation = LaunchConfiguration('enable_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    named_poses_file = LaunchConfiguration('named_poses_file')

    nav_pkg_share = get_package_share_directory('robot_navigation')
    default_named_poses_file = os.path.join(nav_pkg_share, 'config', 'named_poses.yaml')

    # 導航啟用時才拉起 navigation_server
    navigation_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'navigation_server.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'named_poses_file': named_poses_file,
        }.items(),
        condition=IfCondition(enable_navigation),
    )

    fsm_main = Node(
        package='robot_fsm',
        executable='robot_fsm_main',
        name='robot_fsm_main',
        output='screen',
        parameters=[{
            # LaunchConfiguration 預設會被當成字串傳下去，
            # 這裡必須明確指定 value_type=bool，否則節點端 declare_parameter
            # 宣告成 bool 會丟 InvalidParameterTypeException。
            'enable_navigation': ParameterValue(enable_navigation, value_type=bool),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'named_poses_file': named_poses_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_navigation',
            default_value='true',
            description='true = 啟動 navigation_server 並實際導航；'
                        'false = 不啟動，FSM 跳過所有導航與定位步驟（機構測試用）'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'named_poses_file',
            default_value=default_named_poses_file,
            description='Path to named_poses.yaml (world frame poses)'
        ),

        LogInfo(
            msg='[bringup] navigation ENABLED -> launching navigation_server',
            condition=IfCondition(enable_navigation),
        ),
        LogInfo(
            msg='[bringup] navigation DISABLED -> mechanism-only test mode, '
                'navigation_server NOT launched',
            condition=UnlessCondition(enable_navigation),
        ),

        navigation_server,
        fsm_main,
    ])
