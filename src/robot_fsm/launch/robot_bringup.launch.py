# 啟動 FSM 主程式，並依 enable_navigation 決定要不要一併啟動 navigation_server。
#
#   比賽 / 完整模式（預設，導航啟用）：
#     ros2 launch robot_fsm robot_bringup.launch.py
#
#   指定比賽場地（normal / mirror，對應 /workspaces/field_config.yaml 的 sides）：
#     ros2 launch robot_fsm robot_bringup.launch.py field_side:=mirror
#       -> named_poses_file / fsm_side_file 都換成 mirror 的檔案
#       不給 field_side 時用 field_config.yaml 的 default_side
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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_FIELD_CONFIG_FILE = '/workspaces/field_config.yaml'


def launch_side(context):
    enable_navigation = LaunchConfiguration('enable_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    with open(LaunchConfiguration('field_config_file').perform(context)) as f:
        field_config = yaml.safe_load(f)
    field_side = LaunchConfiguration('field_side').perform(context) or field_config['default_side']
    if field_side not in field_config['sides']:
        raise RuntimeError(
            f"field_side '{field_side}' not in field_config.yaml sides "
            f"{list(field_config['sides'])}")
    side = field_config['sides'][field_side]

    nav_pkg_share = get_package_share_directory('robot_navigation')

    # 導航啟用時才拉起 navigation_server
    navigation_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'navigation_server.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'named_poses_file': side['named_poses_file'],
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
            'field_side': field_side,
            'named_poses_file': side['named_poses_file'],
            'fsm_side_file': side['fsm_side_file'],
        }],
    )

    return [
        LogInfo(msg=f'[bringup] field_side = {field_side}'),
        navigation_server,
        fsm_main,
    ]


def generate_launch_description():
    enable_navigation = LaunchConfiguration('enable_navigation')

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
            'field_side',
            default_value='',
            description='normal / mirror（空白 = field_config.yaml 的 default_side）'
        ),
        DeclareLaunchArgument(
            'field_config_file',
            default_value=DEFAULT_FIELD_CONFIG_FILE,
            description='比賽場地統一設定檔'
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

        OpaqueFunction(function=launch_side),
    ])
