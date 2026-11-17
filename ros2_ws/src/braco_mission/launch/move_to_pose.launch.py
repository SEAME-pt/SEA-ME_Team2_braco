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
The move_to_pose mission node, on a running simulation (issue #84) or on the real arm (#101).

Simulation: start it with MoveIt first (braco_moveit_config sim.launch.py, or make up SIM=1 with
SIM_MOVEIT=true), then:

  ros2 launch braco_mission move_to_pose.launch.py

Real arm: the Jetson runs the arm (make up) and the PC runs move_group (make moveit), then on
the PC, on the arm's domain:

  ROS_DOMAIN_ID=22 ros2 launch braco_mission move_to_pose.launch.py real:=true

The node gets the same robot model and MoveIt config as move_group (sim.launch.py or
real.launch.py), so the plans match. real:=true also sets use_control (ADR-0004: take the control
as "mission" before each goal) and the system clock; the simulation has no arm_driver and runs
on simulated time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config(mappings):
    """Return the MoveIt config of braco_moveit_config with the model built with `mappings`."""
    model = os.path.join(get_package_share_directory('braco_description'),
                         'urdf', 'braco.urdf.xacro')
    return (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        .robot_description(file_path=model, mappings=mappings)
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        # Same chain as sim.launch.py: without these the builder looks for default files
        # (e.g. pilz_cartesian_limits.yaml) that braco_moveit_config does not have
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )


def scaled(name):
    """Return the launch argument `name` as a float parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def move_to_pose_node(config, real, condition):
    """Return the move_to_pose node: the real arm (`real`) or the simulation."""
    return Node(
        package='braco_mission', executable='move_to_pose', output='screen',
        condition=condition,
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.joint_limits,
            {
                'use_sim_time': not real,   # the simulation runs on Gazebo's /clock
                'use_control': real,        # ADR-0004: take the control as "mission"
                'velocity_scaling': scaled('velocity_scaling'),
                'position_tolerance_m': scaled('position_tolerance_m'),
                'orientation_tolerance_deg': scaled('orientation_tolerance_deg'),
            },
        ])


def generate_launch_description():
    real = LaunchConfiguration('real')
    # The model Gazebo runs, as in sim.launch.py: with the actuated gripper, so the finger
    # joints of /joint_states are known to the model
    sim_config = moveit_config({'gazebo': 'true', 'gripper_actuated': 'true'})
    # The model the Jetson runs, as in real.launch.py: fingers fixed
    real_config = moveit_config({'real_arm': 'true'})
    return LaunchDescription([
        DeclareLaunchArgument('real', default_value='false',
                              description='true: the real arm (make moveit), false: simulation'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.1',
                              description='MoveIt velocity scaling (ADR-0004: <= 0.1 on the arm)'),
        DeclareLaunchArgument('position_tolerance_m', default_value='0.005',
                              description='success: tcp within this distance of the goal'),
        DeclareLaunchArgument('orientation_tolerance_deg', default_value='2.0',
                              description='success: tcp within this angle of the goal'),
        move_to_pose_node(sim_config, False, UnlessCondition(real)),
        move_to_pose_node(real_config, True, IfCondition(real)),
    ])
