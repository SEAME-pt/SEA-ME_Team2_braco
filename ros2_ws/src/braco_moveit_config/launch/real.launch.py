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
MoveIt 2 on the real braco arm, from the PC (#101): plan in RViz, execute on the Jetson.

The Jetson already runs the arm side (braco.launch.py sim:=false): arm_driver, braco_hardware,
arm_controller and robot_state_publisher. The Jetson image has no MoveIt, so move_group runs
here, on the arm's ROS domain (22), and sends its trajectories to arm_controller over the
network.

First it runs docker/tests/domain-check.sh --target real (#103): right domain, one arm, no
simulation on it. On FAIL the launch shuts down before move_group starts, so MoveIt never takes
a simulation or a second arm for the real one. Read docs/guides/safety.md §9 first.

In the dev container (pc image), on the arm's domain:
  ROS_DOMAIN_ID=22 ros2 launch braco_moveit_config real.launch.py              # RViz window
  ROS_DOMAIN_ID=22 ros2 launch braco_moveit_config real.launch.py rviz:=false  # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
                            Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# The repository is mounted here in the dev and sim containers (docker/compose.yml)
DOMAIN_CHECK = '/opt/braco/docker/tests/domain-check.sh'


def generate_launch_description():
    model = os.path.join(get_package_share_directory('braco_description'),
                         'urdf', 'braco.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        # The model the Jetson runs (braco.launch.py sim:=false): fingers fixed, as the real
        # driver publishes no gripper state
        .robot_description(file_path=model, mappings={'real_arm': 'true'})
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('braco_moveit_config'), 'config', 'moveit.rviz'])

    # No use_sim_time: the real arm runs on the system clock, there is no /clock
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
                      output='screen', parameters=[moveit_config.to_dict()])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', rviz_config],
                parameters=[moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits],
                condition=IfCondition(LaunchConfiguration('rviz')))

    domain_check = ExecuteProcess(
        cmd=['bash', LaunchConfiguration('domain_check'), '--target', 'real'], output='screen')

    def on_check_exit(event, context):
        # Exit status 1 is FAIL, 2 the check could not run: MoveIt starts on PASS only
        if event.returncode != 0:
            return [Shutdown(reason='domain check failed: move_group not started')]
        return [move_group, rviz]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true: RViz with the MotionPlanning panel'),
        DeclareLaunchArgument('domain_check', default_value=DOMAIN_CHECK,
                              description='the domain check script (#103)'),
        domain_check,
        RegisterEventHandler(OnProcessExit(target_action=domain_check,
                                           on_exit=on_check_exit)),
    ])
