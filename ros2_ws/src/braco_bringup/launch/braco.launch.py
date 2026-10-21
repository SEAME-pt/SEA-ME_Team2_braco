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
The braco system in one ROS 2 launch, on the real arm or in simulation.

    sim:=false  arm_driver, temperature_monitor and joystick_teleop (the default). This is
                what the `real` container runs (make up), started by arm_routine.sh, which
                also starts the web page. The arm is not zeroed or parked on start/stop.
    sim:=true   the simulated arm, what the `sim` container on the PC runs. Reserved for
                the Gazebo bringup (#34) with the model from braco_description (#25): until
                then it starts nothing and says so.

Tuning parameters of the nodes live in config/braco.yaml. The wiring (port, baud, lcd_bus,
device) comes from the launch arguments, which the compose file fills from docker/.env.
Also usable by hand in the dev container: ros2 launch braco_bringup braco.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    lcd_bus = LaunchConfiguration('lcd_bus')
    device = LaunchConfiguration('device')
    config = PathJoinSubstitution([FindPackageShare('braco_bringup'), 'config', 'braco.yaml'])
    real = UnlessCondition(sim)
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: simulated arm, false: real arm'),
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud', default_value='1000000'),
        DeclareLaunchArgument('lcd_bus', default_value='/dev/i2c-0'),
        DeclareLaunchArgument('device', default_value='/dev/input/js0'),
        # The only process that opens the serial port. Not respawned: a fault must be seen.
        Node(package='braco_driver', executable='arm_driver', name='arm_driver',
             output='screen', condition=real,
             parameters=[config, {'port': port, 'baud': ParameterValue(baud, value_type=int)}]),
        Node(package='braco_driver', executable='temperature_monitor',
             name='temperature_monitor', output='screen', respawn=True, condition=real,
             parameters=[config, {'lcd_bus': lcd_bus}]),
        Node(package='braco_teleop', executable='joystick_teleop', name='joystick_teleop',
             output='screen', respawn=True, condition=real,
             parameters=[config, {'device': device}]),
        LogInfo(condition=IfCondition(sim),
                msg='sim:=true: the simulated arm is not available yet '
                    '(Gazebo #34, model in braco_description #25), nothing started'),
    ])
