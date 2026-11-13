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
MoveIt 2 on the simulated braco arm: plan in RViz, execute in Gazebo.

Includes braco_simulation's gazebo.launch.py, then starts move_group and RViz once
arm_controller is active, and puts the lab bench's table tops into the planning scene
(scripts/lab_bench_scene.py). Before move_group it runs docker/tests/domain-check.sh
--target sim (#103): not the real arm's domain, one simulation on it; on FAIL it shuts down.

In the dev container (pc image):
  ros2 launch braco_moveit_config sim.launch.py                         # both windows
  ros2 launch braco_moveit_config sim.launch.py gui:=false rviz:=false  # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

DOMAIN_CHECK = '/opt/braco/docker/tests/domain-check.sh'


def is_arm_controller_spawner(action):
    # The spawner lives in the included launch: match it by its command line
    return 'arm_controller' in action.process_description.final_cmd


def generate_launch_description():
    model = os.path.join(get_package_share_directory('braco_description'),
                         'urdf', 'braco.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        # The model Gazebo runs, not config/braco.urdf.xacro (the demo's mock hardware)
        # (gripper_actuated as gazebo.launch.py builds it, so /joint_states' fingers exist here)
        .robot_description(file_path=model,
                           mappings={'gazebo': 'true', 'gripper_actuated': 'true'})
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('braco_moveit_config'), 'config', 'moveit.rviz'])

    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      output='screen', parameters=[moveit_config.to_dict(), sim_time])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', rviz_config],
                parameters=[moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits,
                            sim_time],
                condition=IfCondition(LaunchConfiguration('rviz')))

    # The table tops into move_group's planning scene (it waits for move_group), at the spawn
    # pose gazebo.launch.py placed the arm with: MoveIt then refuses or avoids a pose in the table
    lab_bench_scene = Node(
        package='braco_moveit_config', executable='lab_bench_scene.py', output='screen',
        parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                     for name in ('robot_x', 'robot_y', 'robot_z', 'robot_yaw')}])

    domain_check = ExecuteProcess(
        cmd=['bash', LaunchConfiguration('domain_check'), '--target', 'sim'], output='screen')

    def on_check_exit(event, context):
        if event.returncode != 0:
            return [Shutdown(reason='domain check failed: move_group not started')]
        return [move_group, rviz, lab_bench_scene]

    def on_spawner_exit(event, context):
        # OnProcessExit also fires on failure: without the controller nothing can execute.
        # The check runs once the simulation is up: it counts its /clock and /joint_states
        if event.returncode != 0:
            return [Shutdown(reason='arm_controller did not start')]
        return [domain_check,
                RegisterEventHandler(OnProcessExit(target_action=domain_check,
                                                   on_exit=on_check_exit))]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='true: Gazebo window; false: server only'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true: RViz with the MotionPlanning panel'),
        DeclareLaunchArgument('domain_check', default_value=DOMAIN_CHECK,
                              description='the domain check script (#103)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('braco_simulation'), 'launch', 'gazebo.launch.py'])),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        RegisterEventHandler(OnProcessExit(target_action=is_arm_controller_spawner,
                                           on_exit=on_spawner_exit)),
    ])
