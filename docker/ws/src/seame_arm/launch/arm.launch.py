# Copyright 2026 SEA:ME Team 2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
All arm nodes in one ROS 2 launch: driver, temperature monitor and joystick teleop.

This is what the `arm` container runs (make arm-up), started by arm_routine.sh, which also
moves the arm to zero at start and parks it before stopping. Also usable by hand in the dev
container: ros2 launch seame_arm arm.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    lcd_bus = LaunchConfiguration('lcd_bus')
    device = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud', default_value='1000000'),
        DeclareLaunchArgument('lcd_bus', default_value='/dev/i2c-0'),
        DeclareLaunchArgument('device', default_value='/dev/input/js0'),
        # The only process that opens the serial port. Not respawned: a fault must be seen.
        Node(package='seame_arm', executable='arm_driver', name='arm_driver',
             output='screen',
             parameters=[{'port': port, 'baud': ParameterValue(baud, value_type=int)}]),
        Node(package='seame_arm', executable='temperature_monitor',
             name='temperature_monitor', output='screen', respawn=True,
             parameters=[{'lcd_bus': lcd_bus}]),
        Node(package='seame_arm', executable='joystick_teleop', name='joystick_teleop',
             output='screen', respawn=True, parameters=[{'device': device}]),
    ])
