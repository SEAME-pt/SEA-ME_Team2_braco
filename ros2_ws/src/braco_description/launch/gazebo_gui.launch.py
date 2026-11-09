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
The Gazebo window on a simulation that is already running, close to the arm and tracking it.

Starts no simulation: `make gaze-gui` and `make sim-gui` run it in the dev container, with the
GZ_PARTITION of the `sim` container. config/gazebo_gui.config sets the first view of the camera;
scripts/gz_track_arm.sh turns on Track.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui_config = PathJoinSubstitution(
        [FindPackageShare('braco_description'), 'config', 'gazebo_gui.config'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': ['-g --gui-config ', gui_config]}.items()),
        Node(package='braco_description', executable='gz_track_arm.sh', output='screen'),
    ])
