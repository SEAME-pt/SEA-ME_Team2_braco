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
MoveIt plans and executes on the mock arm (launch/demo.launch.py, headless).

Sends move_group a goal to the SRDF's `park` (not `home`: the mock starts at zero) and checks
the result and that /joint_states ends there.
"""

import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.asserts
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

# Domain 29 is reserved for this test (docker/config/team-domains.tsv); launched nodes inherit it.
os.environ['ROS_DOMAIN_ID'] = '29'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

SHARE = get_package_share_directory('braco_moveit_config')
TARGET = 'park'
TOLERANCE = 0.01  # rad


@pytest.mark.launch_test
def generate_test_description():
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'demo.launch.py')),
        launch_arguments={'rviz': 'false'}.items())
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


def named_state(name):
    srdf = ET.parse(os.path.join(SHARE, 'config', 'braco.srdf')).getroot()
    state = srdf.find("group_state[@name='%s']" % name)
    assert state is not None, 'named state %s missing' % name
    return {j.get('name'): float(j.get('value')) for j in state.findall('joint')}


class TestPlanExecute(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_plan_execute')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, future, timeout, what):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), '%s: no answer in %.0f s' % (what, timeout))
        return future.result()

    def test_plan_and_execute_to_park(self):
        target = named_state(TARGET)
        # move_group starts only once arm_controller is active (demo.launch.py)
        client = ActionClient(self.node, MoveGroup, 'move_action')
        self.assertTrue(client.wait_for_server(timeout_sec=60.0), 'move_group did not start')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 10.0
        goal.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0)
            for j, v in target.items()])]
        goal.planning_options.plan_only = False  # plan and execute

        handle = self.wait(client.send_goal_async(goal), 10.0, 'goal')
        self.assertTrue(handle.accepted, 'move_group rejected the goal')
        result = self.wait(handle.get_result_async(), 60.0, 'result').result
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS,
                         'MoveIt error code %d' % result.error_code.val)

        positions = {}
        self.node.create_subscription(
            JointState, '/joint_states',
            lambda msg: positions.update(zip(msg.name, msg.position)), 10)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(abs(positions.get(j, float('inf')) - v) < TOLERANCE
                   for j, v in target.items()):
                return
        self.fail('/joint_states %s did not reach %s %s' % (positions, TARGET, target))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # 0, or stopped by launch_testing's SIGINT / SIGTERM. Not move_group: it segfaults on
        # SIGINT (MoveIt 2.15, after the test passed), which is not ours to fix.
        for info in proc_info:
            if not info.process_name.startswith('move_group'):
                self.assertIn(info.returncode, [0, -2, -15],
                              '%s exited with %d' % (info.process_name, info.returncode))
