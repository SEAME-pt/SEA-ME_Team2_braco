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
The move_to_pose mission node on a running simulation (issue #84).

Start the simulation with MoveIt first (braco_moveit_config sim.launch.py, or make up SIM=1 with
SIM_MOVEIT=true), then:

  ros2 launch braco_mission move_to_pose.launch.py

The node gets the same robot model and MoveIt config as move_group in sim.launch.py, so the
plans match. use_control stays false: there is no arm_driver in simulation (ADR-0004 sets it
on the real arm).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    model = os.path.join(get_package_share_directory('braco_description'),
                         'urdf', 'braco.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        # The model Gazebo runs, as in sim.launch.py: with the actuated gripper, so the finger
        # joints of /joint_states are known to the model
        .robot_description(file_path=model,
                           mappings={'gazebo': 'true', 'gripper_actuated': 'true'})
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        # Same chain as sim.launch.py: without these the builder looks for default files
        # (e.g. pilz_cartesian_limits.yaml) that braco_moveit_config does not have
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    def arg(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    node = Node(
        package='braco_mission', executable='move_to_pose', output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                'use_sim_time': True,
                'use_control': False,
                'velocity_scaling': arg('velocity_scaling'),
                'position_tolerance_m': arg('position_tolerance_m'),
                'orientation_tolerance_deg': arg('orientation_tolerance_deg'),
            },
        ])

    return LaunchDescription([
        DeclareLaunchArgument('velocity_scaling', default_value='0.1',
                              description='MoveIt velocity scaling (ADR-0004: <= 0.1 on the arm)'),
        DeclareLaunchArgument('position_tolerance_m', default_value='0.005',
                              description='success: tcp within this distance of the goal'),
        DeclareLaunchArgument('orientation_tolerance_deg', default_value='2.0',
                              description='success: tcp within this angle of the goal'),
        node,
    ])
