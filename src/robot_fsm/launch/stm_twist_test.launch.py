from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    stm_test_node = Node(
        package='robot_fsm',
        executable='stm_communication_node',
        name='stm_communication_node',
        output='screen',
        parameters=[{
            'input_cmd_vel_topic': '/unused_cmd_vel',
            'output_cmd_vel_topic': '/mecanum/cmd_vel',
            'cmd_vel_rate_hz': 20.0,
            'cmd_vel_stale_timeout_sec': 0.5,

            'enable_test_twist': True,
            'test_linear_x': 0.03,
            'test_linear_y': 0.0,
            'test_angular_z': 0.0,
            'test_duration_sec': 3.0,

            'enable_mechanism_test_publish': False,
        }]
    )

    return LaunchDescription([
        stm_test_node,
    ])