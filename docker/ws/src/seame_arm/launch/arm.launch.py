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
All arm services in one process tree, for testing inside the dev container.

On the Jetson they normally run as three separate containers (make arm-up).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='seame_arm', executable='arm_driver', name='arm_driver',
             output='screen'),
        Node(package='seame_arm', executable='temperature_monitor',
             name='temperature_monitor', output='screen', respawn=True),
        Node(package='seame_arm', executable='joystick_teleop', name='joystick_teleop',
             output='screen', respawn=True),
    ])
