from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration('publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument('publish_rate_hz', default_value='1.0',
                              description='Talker publish rate in Hz'),

        Node(package='ros2_basics_cpp', executable='talker', name='talker_cpp',
             parameters=[{'publish_rate_hz': rate}], output='screen'),
        Node(package='ros2_basics_py', executable='listener', name='listener_py',
             output='screen'),
        Node(package='ros2_basics_cpp', executable='add_two_ints_server',
             name='add_two_ints_server_cpp', output='screen'),
        Node(package='ros2_basics_py', executable='fibonacci_server',
             name='fibonacci_server_py', output='screen'),
    ])
