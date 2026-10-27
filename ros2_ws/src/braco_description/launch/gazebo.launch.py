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
The simulated braco arm: the model in Gazebo (gz sim), driven by ros2_control.

Starts Gazebo with an empty world, spawns the model (gazebo:=true adds gz_ros2_control),
bridges /clock and starts the controllers of config/controllers.yaml (the same set planned
for the real arm): joint_state_broadcaster, then arm_controller, once the model is in.

braco_bringup's braco.launch.py sim:=true (the `sim` container) includes this file; it also
runs on its own in the dev container:
  ros2 launch braco_description gazebo.launch.py              # Gazebo window
  ros2 launch braco_description gazebo.launch.py gui:=false   # server only (headless)

Move one joint (degrees), or all six (radians, reached in 2 s):
  ros2 run braco_description move_joint.py 2 30
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
      points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')

    share = FindPackageShare('braco_description')
    model = PathJoinSubstitution([share, 'urdf', 'braco.urdf.xacro'])
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model, ' gazebo:=true']),
        value_type=str)

    # gz resolves package://braco_description/... by looking for a braco_description
    # folder on GZ_SIM_RESOURCE_PATH: add the share/ folder that contains it.
    share_parent = os.path.dirname(get_package_share_directory('braco_description'))

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'braco'])
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers])
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--param-file', controllers])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true: Gazebo window; false: server only'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_parent),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [
                PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]),
                '-r -v 1 empty.sdf'],
                # Gazebo gone (crash, window closed) = no arm: end the launch, so the
                # sim container stops like the real one does when arm_driver exits
                'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        spawn,
        # Controllers only once the model (and with it gz_ros2_control) exists
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[arm_controller])),
    ])
