#!/usr/bin/env python3
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
Move the arm in Gazebo through a few poses: an example of code driving the simulation.

Uses the arm_controller action (the interface MoveIt uses too), so each move is waited
for and reports success or failure. Start Gazebo first, then in another shell:

  ros2 launch braco_description gazebo.launch.py
  ros2 run braco_description move_arm_example.py
"""

import math

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Poses in degrees, J1..J6
POSES = [
    [30, 0, 0, 0, 0, 0],
    [30, -20, 40, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
]


class MoveArm(Node):

    def __init__(self, name='move_arm_example'):
        super().__init__(name)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory')
        self.angles = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_joints(self, msg):
        self.angles = dict(zip(msg.name, msg.position))

    def move(self, degrees, seconds=2.0):
        """Go to the six angles (degrees) in `seconds`; True once the arm got there."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in degrees]
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [point]
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('arm_controller not found: is gazebo.launch.py running?')
            return False
        handle = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, handle)
        if not handle.result().accepted:
            return False
        result = handle.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)
        return result.result().result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def current_degrees(self):
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.angles is None:
            return None
        return [round(math.degrees(self.angles[j]), 1) for j in JOINTS]


def main():
    rclpy.init()
    node = MoveArm()
    try:
        for pose in POSES:
            ok = node.move(pose)
            node.get_logger().info('goal %s -> %s, arm at %s'
                                   % (pose, 'ok' if ok else 'FAILED', node.current_degrees()))
            if not ok:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
