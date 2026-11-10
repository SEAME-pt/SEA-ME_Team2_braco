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

Starts Gazebo with the lab bench (worlds/lab_bench.sdf), spawns the model on the table
(gazebo:=true adds gz_ros2_control and the wrist camera), bridges /clock and the camera
(/camera/image_raw, /camera/image_raw/compressed, /camera/camera_info) and starts the
controllers of config/controllers.yaml (the same set planned for the real arm):
joint_state_broadcaster, then arm_controller, once the model is in, then
gripper_action_controller (simulation only: the model is built with gripper_actuated:=true, the
real arm keeps its fingers fixed).

The model's `world` frame is the table surface, so its spawn pose is where the arm stands in the
scene: robot_x and robot_y on the table, robot_z its height, robot_yaw about z (0: the arm's front
along the gap, towards the table top). TF and MoveIt keep working in the model's frames.

braco_bringup's braco.launch.py sim:=true (the `sim` container) includes this file; it also
runs on its own in the dev container. It refuses to start when a /controller_manager already
runs on the ROS domain (the `sim` container is up: `make down`, or use that one):
  ros2 launch braco_simulation gazebo.launch.py              # Gazebo window
  ros2 launch braco_simulation gazebo.launch.py gui:=false   # server only (headless)
  ros2 launch braco_simulation gazebo.launch.py robot_x:=0.2 robot_yaw:=0.5

Close and open the gripper (radians of its master joint: -0.74 closed .. 0.15 open):
  ros2 action send_goal /gripper_action_controller/gripper_cmd \
    control_msgs/action/ParallelGripperCommand \
    "{command: {name: [gripper_controller], position: [-0.74]}}"

Move one joint (degrees), or all six (radians, reached in 2 s):
  ros2 run braco_simulation move_joint.py 2 30
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
      points: [{positions: [0.5, 0.3, -0.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import rclpy

# DDS discovery finds a running controller_manager in ~0.2 s; when there is none the check waits
# this long, once, before Gazebo starts.
CHECK_S = 2.0


def controller_manager_running():
    """
    Tell whether a /controller_manager already runs on this ROS domain.

    That is another simulation (the `sim` container, a second launch) or the real arm's
    controllers. Started anyway, this launch's spawners would reach that controller_manager, fail
    on its already active controllers, end the launch and leave this Gazebo server running.
    """
    context = rclpy.Context()
    rclpy.init(args=[], context=context)  # not sys.argv: the launch arguments are no remaps
    try:
        node = rclpy.create_node('gazebo_launch_check', context=context)
        end = time.monotonic() + CHECK_S
        while time.monotonic() < end:
            if ('controller_manager', '/') in node.get_node_names_and_namespaces():
                return True
            time.sleep(0.1)
        return False
    finally:
        rclpy.shutdown(context=context)


def generate_launch_description():
    gui = LaunchConfiguration('gui')

    # The robot model and the controllers: braco_description
    share = FindPackageShare('braco_description')
    model = PathJoinSubstitution([share, 'urdf', 'braco.urdf.xacro'])
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', model,
                 ' gazebo:=true gripper_actuated:=true']),
        value_type=str)
    # The scene and the window: this package
    gui_config = PathJoinSubstitution(
        [FindPackageShare('braco_simulation'), 'config', 'gazebo_gui.config'])

    # gz resolves package://braco_description/... by looking for a braco_description
    # folder on GZ_SIM_RESOURCE_PATH: add the share/ folder that contains it, and models/ for
    # the world's model://lab_table.
    share_parent = os.path.dirname(get_package_share_directory('braco_description'))
    sim_dir = get_package_share_directory('braco_simulation')
    models = os.path.join(sim_dir, 'models')
    world = os.path.join(sim_dir, 'worlds', 'lab_bench.sdf')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'braco',
                            '-x', LaunchConfiguration('robot_x'),
                            '-y', LaunchConfiguration('robot_y'),
                            '-z', LaunchConfiguration('robot_z'),
                            '-Y', LaunchConfiguration('robot_yaw')])
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers])
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', '--param-file', controllers])
    gripper_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_action_controller', '--param-file', controllers])

    arguments = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='true: Gazebo window; false: server only'),
        DeclareLaunchArgument('world', default_value=world,
                              description='world file, e.g. empty.sdf (then set robot_z:=0)'),
        # Default: the mount's rear edge flush with the table's (0.07 = half its depth), on the
        # table at y > 0 next to the gap (0.085 = half its width + half the gap). UNMEASURED.
        DeclareLaunchArgument('robot_x', default_value='0.07',
                              description='arm position along x in the world, metres'),
        DeclareLaunchArgument('robot_y', default_value='0.085',
                              description='arm position along y in the world, metres'),
        DeclareLaunchArgument('robot_z', default_value='0.75',
                              description='height of the surface the arm stands on, metres'),
        DeclareLaunchArgument('robot_yaw', default_value='0.0',
                              description='arm rotation about z in the world, radians'),
    ]
    simulation = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share_parent),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [
                PythonExpression(["'' if '", gui, "' == 'true' else '-s '"]),
                '-r -v 1 --gui-config ', gui_config, ' ', LaunchConfiguration('world')],
                # Gazebo gone (crash, window closed) = no arm: end the launch, so the
                # sim container stops like the real one does when arm_driver exits
                'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']),
        # The wrist camera's images, through image_transport: /camera/image_raw and its
        # /compressed variant, the topic the real camera publishes (and the web page reads)
        Node(package='ros_gz_image', executable='image_bridge', output='screen',
             arguments=['/camera/image_raw']),
        spawn,
        # The window starts tracking the arm (gui:=true only)
        Node(package='braco_simulation', executable='gz_track_arm.sh', output='screen',
             condition=IfCondition(gui)),
        # Controllers only once the model (and with it gz_ros2_control) exists
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster,
                                           on_exit=[arm_controller])),
        RegisterEventHandler(OnProcessExit(target_action=arm_controller,
                                           on_exit=[gripper_controller])),
    ]

    def start(context):
        if controller_manager_running():
            return [LogInfo(msg='Another simulation (or the real arm controllers) is already '
                                'running on ROS_DOMAIN_ID %s: its /controller_manager would take '
                                'the controllers of this one. Stop it first (the sim container: '
                                'make down), or use it.' % os.environ.get('ROS_DOMAIN_ID', '0')),
                    Shutdown(reason='another controller_manager is running')]
        return simulation

    return LaunchDescription([*arguments, OpaqueFunction(function=start)])
