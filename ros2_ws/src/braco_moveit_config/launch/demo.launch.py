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
MoveIt 2 demo of the braco arm: plan and execute in RViz, no Gazebo and no real arm.

Starts the model (config/braco.urdf.xacro: braco_description + mock_components), ros2_control
with the controllers of braco_description/config/controllers.yaml (joint_state_broadcaster,
arm_controller), move_group with the config/ of this package, and RViz with the
MotionPlanning panel. The mock hardware reports back the position it is commanded, so an
executed plan "moves" the arm at once. move_group and RViz start once arm_controller is
active, so the first goal can execute; if it fails to start, the launch shuts down.

In the dev container (pc image):
  ros2 launch braco_moveit_config demo.launch.py              # RViz window
  ros2 launch braco_moveit_config demo.launch.py rviz:=false  # headless (move_group only)

In RViz, MotionPlanning > Planning: Goal State `home` or `park`, then Plan & Execute.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('braco', package_name='braco_moveit_config')
        .robot_description(file_path='config/braco.urdf.xacro')
        .robot_description_semantic(file_path='config/braco.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        # OMPL only: CHOMP, STOMP and Pilz are not configured for the arm.
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # The one controller set of the arm (the same in Gazebo and, planned, on the real arm).
    controllers = PathJoinSubstitution(
        [FindPackageShare('braco_description'), 'config', 'controllers.yaml'])
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('braco_moveit_config'), 'config', 'moveit.rviz'])

    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers])
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--param-file', controllers])

    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[moveit_config.to_dict()])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', rviz_config],
                parameters=[moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits],
                condition=IfCondition(LaunchConfiguration('rviz')))

    def on_arm_controller_exit(event, context):
        # OnProcessExit also fires on failure: without the controller nothing can execute
        if event.returncode != 0:
            return [Shutdown(reason='arm_controller did not start')]
        return [move_group, rviz]

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true: RViz with the MotionPlanning panel'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[moveit_config.robot_description]),
        # controller_manager reads the model (and its mock <ros2_control>) from /robot_description
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers],
             remappings=[('~/robot_description', '/robot_description')]),
        joint_state_broadcaster,
        # arm_controller after joint_state_broadcaster, as in braco_description's Gazebo launch
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[arm_controller])),
        # move_group after arm_controller, as in sim.launch.py
        RegisterEventHandler(OnProcessExit(target_action=arm_controller,
                                           on_exit=on_arm_controller_exit)),
    ])
