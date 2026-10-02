# ros2 launch robot_navigation named_pose_recorder.launch.py field_side:=mirror
#   -> 寫入 /workspaces/field_config.yaml 中 sides.mirror.named_poses_file（src/ 底下）
#   不給 field_side 時用 default_side；也可以直接指定 named_poses_file:=<path>
#
# 錄完新的點之後，讓 navigation_server 熱重載（不用重啟）：
#   ros2 service call /reload_named_poses std_srvs/srv/Trigger

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

FIELD_CONFIG_FILE = '/workspaces/field_config.yaml'


def recorder(context):
    named_poses_file = LaunchConfiguration('named_poses_file').perform(context)
    if not named_poses_file:
        with open(FIELD_CONFIG_FILE) as f:
            field_config = yaml.safe_load(f)
        field_side = LaunchConfiguration('field_side').perform(context) or field_config['default_side']
        named_poses_file = field_config['sides'][field_side]['named_poses_file']

    return [
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
                    'stage2_push_clam',
                    'stage2_move_forward_align',
                    'stage2_return_point',
                    'stage3_entry',
                    'stage3_pick_pose',
                    'stage3_stack_pose',
                ],
            }]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'field_side',
            default_value='',
            description='normal / mirror（空白 = field_config.yaml 的 default_side）'
        ),
        DeclareLaunchArgument(
            'named_poses_file',
            default_value='',
            description='直接指定要寫入的 named_poses yaml（空白 = 依 field_side）'
        ),
        OpaqueFunction(function=recorder),
    ])
