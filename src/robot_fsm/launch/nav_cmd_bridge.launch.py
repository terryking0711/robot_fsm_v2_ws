from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cmd_bridge_node = Node(
        package='robot_fsm',
        executable='stm_communication_node',
        name='stm_communication_node',
        output='screen',
        parameters=[{
            'input_cmd_vel_topic': '/cmd_vel',
            'output_cmd_vel_topic': '/mecanum/cmd_vel',
            'cmd_vel_rate_hz': 20.0,
            'cmd_vel_stale_timeout_sec': 0.5,
            'enable_test_twist': False,
            'enable_mechanism_test_publish': False,
        }]
    )

    return LaunchDescription([
        cmd_bridge_node,
    ])