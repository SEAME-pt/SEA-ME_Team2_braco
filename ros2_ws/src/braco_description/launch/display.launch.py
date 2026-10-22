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
Show the braco model in RViz2.

Starts robot_state_publisher, RViz and, with use_gui:=true, the joint_state_publisher_gui sliders.

  Simulation only (sliders):  ros2 launch braco_description display.launch.py
  Following the real arm:     ros2 launch braco_description display.launch.py use_gui:=false
                              (on the PC; the Jetson's arm_driver streams /joint_states over DDS)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    gripper_actuated = LaunchConfiguration('gripper_actuated')
    rviz = LaunchConfiguration('rviz')

    share = FindPackageShare('braco_description')
    model = PathJoinSubstitution([share, 'urdf', 'braco.urdf.xacro'])
    rviz_config = PathJoinSubstitution([share, 'rviz', 'braco.rviz'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model,
                 ' gripper_actuated:=', gripper_actuated]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true',
                              description='true: joint_state_publisher_gui sliders (simulation); '
                                          'false: follow the real /joint_states'),
        DeclareLaunchArgument('gripper_actuated', default_value='false',
                              description='true: vendor finger linkage with a slider'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz2'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(use_gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(rviz)),
    ])
