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
#   略過第二關（STAGE2_CLAM），開始後直接去 stage3_entry：
#     預設看 field_config.yaml 該 side 的 skip_stage2（dit = true），
#     ros2 launch robot_fsm robot_bringup.launch.py field_side:=dit skip_stage2:=false
#       -> 手動覆蓋
#
#   第二關關閉手臂 state（2000/2001/203/204/205/206/111/2021/207 及其 mirror 代號都不送，
#   連 state 都不進入；LIFT_ARM 後直接導航到 stage2_return_point 再收手臂）：
#     ros2 launch robot_fsm robot_bringup.launch.py skip_stage2_arm:=true
#
#   只跑第三關（START 之後不跑第二關，
#   直接從目前位置 Nav2 到 stage3_entry 再跑第三關）：
#     ros2 launch robot_fsm robot_bringup.launch.py field_side:=dit only_stage3:=true
#
#   RESET：保持現有定位與截止時間（截止時間由 fsm_launcher 傳入）。
#     reset_stage2:=true -> 重跑第二關到第三關
#     reset_stage3:=true -> DirectTransition 轉角 -> stage3_entry -> 第三關
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

    # skip_stage2 launch arg 空白 -> 用 field_config.yaml 該 side 的設定
    skip_stage2_arg = LaunchConfiguration('skip_stage2').perform(context).strip().lower()
    if skip_stage2_arg == '':
        skip_stage2 = bool(side.get('skip_stage2', False))
    elif skip_stage2_arg in ('true', 'false'):
        skip_stage2 = skip_stage2_arg == 'true'
    else:
        raise RuntimeError(f"skip_stage2 must be true / false, got '{skip_stage2_arg}'")

    skip_stage2_arm_arg = LaunchConfiguration('skip_stage2_arm').perform(context).strip().lower()
    if skip_stage2_arm_arg not in ('true', 'false'):
        raise RuntimeError(f"skip_stage2_arm must be true / false, got '{skip_stage2_arm_arg}'")
    skip_stage2_arm = skip_stage2_arm_arg == 'true'

    only_stage3_arg = LaunchConfiguration('only_stage3').perform(context).strip().lower()
    if only_stage3_arg not in ('true', 'false'):
        raise RuntimeError(f"only_stage3 must be true / false, got '{only_stage3_arg}'")
    only_stage3 = only_stage3_arg == 'true'

    motion_test_arg = LaunchConfiguration('stage3_motion_test').perform(context).strip().lower()
    if motion_test_arg not in ('true', 'false'):
        raise RuntimeError('stage3_motion_test must be true / false')
    stage3_motion_test = motion_test_arg == 'true'
    if stage3_motion_test and field_side != 'dit':
        raise RuntimeError('stage3_motion_test requires field_side:=dit')

    reset_stage2_arg = LaunchConfiguration('reset_stage2').perform(context).strip().lower()
    if reset_stage2_arg not in ('true', 'false'):
        raise RuntimeError(f"reset_stage2 must be true / false, got '{reset_stage2_arg}'")
    reset_stage2 = reset_stage2_arg == 'true'

    reset_stage3_arg = LaunchConfiguration('reset_stage3').perform(context).strip().lower()
    if reset_stage3_arg not in ('true', 'false'):
        raise RuntimeError(f"reset_stage3 must be true / false, got '{reset_stage3_arg}'")
    reset_stage3 = reset_stage3_arg == 'true'
    if stage3_motion_test and (reset_stage2 or reset_stage3):
        raise RuntimeError('stage3_motion_test cannot be combined with RESET')
    if reset_stage2 and reset_stage3:
        raise RuntimeError('reset_stage2 and reset_stage3 are mutually exclusive')
    if reset_stage2:
        only_stage3, skip_stage2, skip_stage2_arm = False, False, True

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
            'mirror_v2': ParameterValue(LaunchConfiguration('mirror_v2'), value_type=bool),
            'field_side': field_side,
            'match_deadline_unix_sec': ParameterValue(
                LaunchConfiguration('match_deadline_unix_sec'), value_type=float),
            'named_poses_file': side['named_poses_file'],
            'fsm_side_file': side['fsm_side_file'],
            'skip_stage2': skip_stage2,
            'skip_stage2_arm': skip_stage2_arm,
            'only_stage3': only_stage3,
            'stage3_motion_test': stage3_motion_test,
            'reset_stage2': reset_stage2,
            'reset_stage3': reset_stage3,
        }],
    )

    return [
        LogInfo(msg=f'[bringup] field_side = {field_side}, skip_stage2 = {skip_stage2}, '
                    f'only_stage3 = {only_stage3}, reset_stage2 = {reset_stage2}, reset_stage3 = {reset_stage3}'),
        *([] if stage3_motion_test else [navigation_server]),
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
            'skip_stage2',
            default_value='',
            description='true / false = 略過第二關直接去第三關（空白 = field_config.yaml 該 side 的 skip_stage2）'
        ),
        DeclareLaunchArgument(
            'skip_stage2_arm',
            default_value='false',
            description='true = 送2101後直接到stage2_entry，保留第二段伸臂、推動、抬臂；'
                        '抬臂後直接到stage2_return_point收臂'
        ),
        DeclareLaunchArgument('mirror_v2', default_value='false',
                              description='Mirror V2: s3_entry_v2, rows 3 -> 1 -> 2'),
        DeclareLaunchArgument(
            'only_stage3',
            default_value='false',
            description='true = 只跑第三關：START 後直接 Nav2 到 stage3_entry 跑第三關'
                        '（不跑第二關）'
        ),
        DeclareLaunchArgument(
            'stage3_motion_test', default_value='false',
            description='DIT only: spin -> forward -> direct place -> stop, no mechanism commands'
        ),
        DeclareLaunchArgument(
            'reset_stage2', default_value='false',
            description='Rerun stage 2 -> stage 3, keep localization and supplied match deadline'
        ),
        DeclareLaunchArgument(
            'reset_stage3',
            default_value='false',
            description='true = 保留定位，DirectTransition 到轉角再到 stage3_entry 跑第三關'
        ),
        DeclareLaunchArgument(
            'match_deadline_unix_sec',
            default_value='0.0',
            description='START NORMAL / MIRROR absolute match deadline; 0 disables the match timer'
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
