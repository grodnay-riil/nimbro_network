from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='localhost',
            description='Destination hostname or IP address'),
        DeclareLaunchArgument('port', default_value='17001',
            description='Destination port'),
        DeclareLaunchArgument('config', default_value='',
            description='Path to YAML config file (optional)'),

        Node(
            package='nimbro_topic_transport',
            executable='udp_sender',
            name='udp_sender',
            output='screen',
            parameters=[
                {'destination_addr': LaunchConfiguration('target')},
                {'destination_port': LaunchConfiguration('port')},
                LaunchConfiguration('config'),
            ],
        ),
    ])
