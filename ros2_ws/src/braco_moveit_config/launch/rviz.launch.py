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
RViz with the MotionPlanning panel, on the move_group of the simulation already running.

Starts neither a simulation nor move_group: `make rviz-gui` runs it in the dev container, on
the ROS domain of the `sim` container (which runs move_group with SIM_MOVEIT=true).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    model = os.path.join(get_package_share_directory('braco_description'),
                         'urdf', 'braco.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        # The model Gazebo runs, as in sim.launch.py: with the actuated gripper, or RViz dies on
        # move_group's first planning scene (its finger joints are unknown to the model)
        .robot_description(file_path=model,
                           mappings={'gazebo': 'true', 'gripper_actuated': 'true'})
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('braco_moveit_config'), 'config', 'moveit.rviz'])

    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', rviz_config],
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         moveit_config.planning_pipelines,
                         moveit_config.joint_limits,
                         {'use_sim_time': True}]),
    ])
