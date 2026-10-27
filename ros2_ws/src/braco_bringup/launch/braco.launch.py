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

    sim:=false  arm_driver, temperature_monitor, joystick_teleop and camera (the default). This is
                what the `real` container runs (make up), started by arm_routine.sh, which
                also starts the web page. The arm is not zeroed or parked on start/stop.
                arm_driver is not respawned: when it exits the whole launch shuts down, so
                arm_routine.sh ends and the container stops (a driver fault is always seen).
    sim:=true   the simulated arm (PC only), what the `sim` container runs: Gazebo with the
                braco_description model and the ros2_control controllers planned for the
                real arm (braco_description's gazebo.launch.py). When Gazebo exits the launch
                shuts down, like with arm_driver. gui:=true opens the Gazebo window.

Tuning parameters of the nodes live in config/braco.yaml. The wiring (port, baud, lcd_bus,
device, camera) comes from the launch arguments, which the compose file fills from docker/.env.
Also usable by hand in the dev container: ros2 launch braco_bringup braco.launch.py
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    camera = LaunchConfiguration('camera')
    config = PathJoinSubstitution([FindPackageShare('braco_bringup'), 'config', 'braco.yaml'])
    real = UnlessCondition(sim)
    # Not an exec_depend: braco_description needs Gazebo, which only the PC image has. The
    # package is looked up only when sim:=true, so the Jetson never needs it.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('braco_description'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
        condition=IfCondition(sim))
    # The only process that opens the serial port. Not respawned: a fault must be seen.
    arm_driver = Node(package='braco_driver', executable='arm_driver', name='arm_driver',
                      output='screen', condition=real,
                      parameters=[config, {'port': port,
                                           'baud': ParameterValue(baud, value_type=int)}])
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true: simulated arm, false: real arm'),
        DeclareLaunchArgument('port', default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('baud', default_value='1000000'),
        DeclareLaunchArgument('lcd_bus', default_value='/dev/i2c-0'),
        DeclareLaunchArgument('device', default_value='/dev/input/js0'),
        DeclareLaunchArgument('camera', default_value='/dev/video0',
                              description='video device of the eye-in-hand camera'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='sim:=true only: true opens the Gazebo window'),
        arm_driver,
        # Without the driver nothing else is useful: end the launch instead of idling
        RegisterEventHandler(
            OnProcessExit(target_action=arm_driver,
                          on_exit=[Shutdown(reason='arm_driver exited')]),
            condition=real),
        Node(package='braco_driver', executable='temperature_monitor',
             name='temperature_monitor', output='screen', respawn=True, condition=real,
             parameters=[config, {'lcd_bus': lcd_bus}]),
        Node(package='braco_teleop', executable='joystick_teleop', name='joystick_teleop',
             output='screen', respawn=True, condition=real,
             parameters=[config, {'device': device}]),
        # Never needed by the arm: without a camera it waits for one
        Node(package='braco_camera', executable='camera', name='camera',
             output='screen', respawn=True, condition=real,
             parameters=[config, {'device': camera}]),
        gazebo,
    ])
