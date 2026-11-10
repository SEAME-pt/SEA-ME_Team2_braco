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
Put the lab bench's table tops into move_group's planning scene, so MoveIt plans around them.

The tables come from the files Gazebo loads: braco_simulation's worlds/lab_bench.sdf (where
each table stands) and models/lab_table/model.sdf (its top). They are moved into the model's
`world` frame, the table surface under the arm, with the arm's spawn pose in Gazebo: the
robot_x, robot_y, robot_z and robot_yaw parameters, the arguments of gazebo.launch.py. So a
measured table (UNMEASURED today) reaches MoveIt without a second copy of its sizes. Only the
tops: the legs are under them, out of the arm's reach.

The top's surface is z = 0, the table itself, not the 20 mm margin of the real arm's
table-clearance check (TABLE_MARGIN_MM, mycobot.hpp): arm_driver still refuses the last 20 mm.

sim.launch.py runs it once move_group is up; it adds the objects and exits. The scene lives in
move_group, so after a move_group restart run it again:
  ros2 run braco_moveit_config lab_bench_scene.py --ros-args \
    -p robot_x:=0.07 -p robot_y:=0.085 -p robot_z:=0.75 -p robot_yaw:=0.0
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

TABLE_URI = 'model://lab_table'
WAIT_FOR_MOVE_GROUP_S = 120.0


def floats(text):
    return [float(v) for v in text.split()]


def table_tops(world_sdf, table_sdf, robot):
    """
    Each table top of the world, in the model's `world` frame.

    robot is the arm's spawn pose in Gazebo, (x, y, z, yaw). Returns a list of
    (name, (x, y, z) of the top's centre, yaw, (size x, size y, size z)).
    """
    top = ET.parse(table_sdf).getroot().find(".//collision[@name='top_collision']")
    size = floats(top.find('geometry/box/size').text)
    local = floats(top.find('pose').text)   # in the table's frame, not rotated
    robot_x, robot_y, robot_z, robot_yaw = robot
    tops = []
    for include in ET.parse(world_sdf).getroot().iter('include'):
        if include.find('uri').text.strip() != TABLE_URI:
            continue
        x, y, z, _, _, yaw = floats(include.find('pose').text)
        # The top's centre in Gazebo, then relative to the arm and turned by -robot_yaw
        dx = x + local[0] * math.cos(yaw) - local[1] * math.sin(yaw) - robot_x
        dy = y + local[0] * math.sin(yaw) + local[1] * math.cos(yaw) - robot_y
        cos, sin = math.cos(-robot_yaw), math.sin(-robot_yaw)
        centre = (cos * dx - sin * dy, sin * dx + cos * dy, z + local[2] - robot_z)
        tops.append((include.find('name').text.strip(), centre, yaw - robot_yaw, size))
    return tops


def main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    from rclpy.parameter import Parameter
    from shape_msgs.msg import SolidPrimitive

    rclpy.init()
    node = rclpy.create_node('lab_bench_scene')
    robot = []
    for name in ('robot_x', 'robot_y', 'robot_z', 'robot_yaw'):
        node.declare_parameter(name, Parameter.Type.DOUBLE)
        robot.append(node.get_parameter(name).value)
    if None in robot:
        node.get_logger().error('robot_x, robot_y, robot_z and robot_yaw must all be set')
        return 1

    share = get_package_share_directory('braco_simulation')
    tops = table_tops(os.path.join(share, 'worlds', 'lab_bench.sdf'),
                      os.path.join(share, 'models', 'lab_table', 'model.sdf'), robot)

    scene = PlanningScene(is_diff=True)
    for name, centre, yaw, size in tops:
        table = CollisionObject(id=name, operation=CollisionObject.ADD)
        table.header.frame_id = 'world'
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
        table.pose.position.x, table.pose.position.y, table.pose.position.z = centre
        table.pose.orientation.z = math.sin(yaw / 2)
        table.pose.orientation.w = math.cos(yaw / 2)
        table.primitive_poses = [Pose()]   # the box at the object's pose (identity)
        table.primitive_poses[0].orientation.w = 1.0
        scene.world.collision_objects.append(table)

    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=WAIT_FOR_MOVE_GROUP_S):
        node.get_logger().error('no move_group (/apply_planning_scene) after %.0f s'
                                % WAIT_FOR_MOVE_GROUP_S)
        return 1
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future)
    if not future.result() or not future.result().success:
        node.get_logger().error('move_group did not apply the table tops')
        return 1
    for name, centre, _, size in tops:
        node.get_logger().info('%s: %.2f x %.2f m, top at z = %.3f m in world'
                               % (name, size[0], size[1], centre[2] + size[2] / 2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
