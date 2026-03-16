from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='17001',
            description='Port to listen on'),

        Node(
            package='nimbro_topic_transport',
            executable='tcp_receiver',
            name='tcp_receiver',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
            ],
        ),
    ])
