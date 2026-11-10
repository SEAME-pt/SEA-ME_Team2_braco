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
Keep the URDF and the code in sync.

Processes braco.urdf.xacro and checks that

  * the arm joints are named joint1..joint6, as arm_driver publishes them in /joint_states
  * the <limit> of every joint is JOINT_LIMITS of ros2_ws/src/mycobot_cpp/includes/mycobot.hpp
  * every link with geometry has a positive mass and a positive-definite inertia
  * the frames the system uses exist (world, base_link, tool0, tcp, camera_optical_frame)
  * every mesh file, and every texture image a mesh points to, exists
  * every mesh declares the COLLADA unit 1 (MoveIt ignores the unit; the URDF scales instead)
  * the model's zero pose has not moved by accident (see ZERO_TOOL0)
  * gazebo:=true controls exactly joint1..joint6, and the plain model has no ros2_control
  * config/controllers.yaml: arm_controller drives joint1..joint6
  * gazebo:=true has the wrist camera at camera_link, sized like the real camera
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MYCOBOT_HPP_CANDIDATES = [
    os.environ.get('MYCOBOT_HPP', ''),
    # the mycobot_cpp package next to this one (ros2_ws/src/mycobot_cpp; /ws/src in the container)
    os.path.join(PACKAGE_DIR, '..', 'mycobot_cpp', 'includes', 'mycobot.hpp'),
]
ARM_JOINTS = ['joint%d' % n for n in range(1, 7)]

# tool0 position at all joints = 0 (metres, in world), from the vendor joint origins and the
# G-base height. NOT checked against the real arm yet: that is the zero-pose comparison of
# docs/hardware/urdf-sim-to-real.md. This only catches an accidental edit of the origins.
ZERO_TOOL0 = (0.0456, -0.06462, 0.030 + 0.43614)   # G-base + arm


def load_urdf(gripper_actuated='false', gazebo='false'):
    """Process braco.urdf.xacro without needing the workspace to be sourced."""
    path = os.path.join(PACKAGE_DIR, 'urdf', 'braco.urdf.xacro')
    with open(path, encoding='utf-8') as f:
        text = f.read().replace('$(find braco_description)', PACKAGE_DIR)
    doc = xacro.parse(text)
    xacro.process_doc(doc, mappings={'gripper_actuated': gripper_actuated, 'gazebo': gazebo})
    return ET.fromstring(doc.toxml())


def floats(text):
    return [float(v) for v in text.split()]


def mycobot_source(filename):
    """Return the text of mycobot_cpp includes/mycobot.hpp or srcs/mycobot.cpp."""
    for candidate in MYCOBOT_HPP_CANDIDATES:
        if candidate and os.path.isfile(candidate):
            if filename != 'mycobot.hpp':
                candidate = os.path.join(os.path.dirname(candidate), '..', 'srcs', filename)
            if os.path.isfile(candidate):
                with open(candidate, encoding='utf-8') as f:
                    return f.read()
    pytest.skip('%s not found (set MYCOBOT_HPP)' % filename)


def hpp_constant(name):
    """Return `constexpr double <name>` of mycobot.hpp."""
    match = re.search(r'constexpr double ' + name + r'\s*=\s*(-?[\d.]+)',
                      mycobot_source('mycobot.hpp'))
    assert match, '%s not found in mycobot.hpp' % name
    return float(match.group(1))


def hpp_limits(name):
    """Return the {min, max} pairs of `constexpr JointLimits <name>` in mycobot.hpp, in degrees."""
    text = mycobot_source('mycobot.hpp')
    block = re.search(name + r'\s*=\s*\{\{(.*?)\}\};', text, re.S)
    assert block, '%s not found in mycobot.hpp' % name
    pairs = re.findall(r'\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}', block.group(1))
    assert len(pairs) == 6
    return [(float(lo), float(hi)) for lo, hi in pairs]


def test_arm_joints_are_joint1_to_joint6():
    robot = load_urdf()
    revolute = [j.get('name') for j in robot.findall('joint')
                if j.get('type') == 'revolute' and not j.get('name').startswith('gripper')]
    assert revolute == ARM_JOINTS


def test_limits_match_mycobot_hpp():
    robot = load_urdf()
    expected = hpp_limits('JOINT_LIMITS')
    for name, (lo, hi) in zip(ARM_JOINTS, expected):
        limit = robot.find("joint[@name='%s']/limit" % name)
        assert float(limit.get('lower')) == pytest.approx(math.radians(lo), abs=1e-9), name
        assert float(limit.get('upper')) == pytest.approx(math.radians(hi), abs=1e-9), name


def test_base_height_matches_mycobot_hpp():
    # The C++ table check (minHeightMm) starts its chain at the bottom of the casing,
    # BASE_HEIGHT_MM - CASING_HEIGHT_MM above the table: the same point as base_link here.
    robot = load_urdf()
    z = floats(robot.find("joint[@name='g_base_to_base_link']/origin").get('xyz'))[2]
    expected = (hpp_constant('BASE_HEIGHT_MM') - hpp_constant('CASING_HEIGHT_MM')) / 1000.0
    assert z == pytest.approx(expected, abs=0.001)


def test_joint_origins_match_mycobot_cpp():
    # minHeightMm keeps the arm off the table with its own copy of the kinematics.
    block = re.search(r'JOINT_ORIGINS\[6\]\s*=\s*\{(.*?)\};', mycobot_source('mycobot.cpp'), re.S)
    assert block, 'JOINT_ORIGINS not found in mycobot.cpp'
    cpp = re.findall(r'\{\{\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\}', block.group(1))
    assert len(cpp) == 6
    robot = load_urdf()
    for name, xyz in zip(ARM_JOINTS, cpp):
        urdf = floats(robot.find("joint[@name='%s']/origin" % name).get('xyz'))
        assert urdf == pytest.approx([float(v) for v in xyz], abs=1e-6), name


def test_park_script_matches_mycobot_hpp():
    # scripts/park.py parks the Gazebo arm where the real arm parks.
    with open(os.path.join(PACKAGE_DIR, 'scripts', 'park.py'), encoding='utf-8') as f:
        match = re.search(r'PARK_DEG = \[(.*?)\]', f.read())
    assert match, 'PARK_DEG not found in scripts/park.py'
    script = [float(v) for v in match.group(1).split(',')]
    hpp = re.search(r'PARK_ANGLES = \{(.*?)\};', mycobot_source('mycobot.hpp'))
    assert hpp, 'PARK_ANGLES not found in mycobot.hpp'
    assert script == [float(v) for v in hpp.group(1).split(',')]


def test_move_joint_limits_match_mycobot_hpp():
    # scripts/move_joint.py refuses an angle the real arm would clamp.
    with open(os.path.join(PACKAGE_DIR, 'scripts', 'move_joint.py'), encoding='utf-8') as f:
        match = re.search(r'LIMITS_DEG = \[(.*?)\]\n', f.read())
    assert match, 'LIMITS_DEG not found in scripts/move_joint.py'
    pairs = re.findall(r'\(\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\)', match.group(1))
    assert [(float(lo), float(hi)) for lo, hi in pairs] == hpp_limits('JOINT_LIMITS')


def test_every_link_with_geometry_has_positive_mass_and_inertia():
    robot = load_urdf()
    for link in robot.findall('link'):
        name = link.get('name')
        if link.find('visual') is None and link.find('collision') is None:
            continue
        inertial = link.find('inertial')
        assert inertial is not None, '%s has geometry but no <inertial>' % name
        assert float(inertial.find('mass').get('value')) > 0.0, name
        i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
        # positive definite: every leading minor > 0 (matrix is symmetric)
        m = [[i['ixx'], i['ixy'], i['ixz']],
             [i['ixy'], i['iyy'], i['iyz']],
             [i['ixz'], i['iyz'], i['izz']]]
        det2 = m[0][0] * m[1][1] - m[0][1] ** 2
        det3 = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] ** 2)
                - m[0][1] * (m[0][1] * m[2][2] - m[1][2] * m[0][2])
                + m[0][2] * (m[0][1] * m[1][2] - m[1][1] * m[0][2]))
        assert m[0][0] > 0 and det2 > 0 and det3 > 0, '%s inertia is not positive definite' % name


@pytest.mark.parametrize('frame', ['world', 'g_base', 'base_link', 'link6', 'tool0', 'tcp',
                                   'camera_mount', 'camera_link', 'camera_optical_frame'])
def test_frame_exists(frame):
    robot = load_urdf()
    assert robot.find("link[@name='%s']" % frame) is not None


def test_tree_is_connected():
    """Every link except `world` is the child of exactly one joint (a single tree)."""
    robot = load_urdf()
    links = {link.get('name') for link in robot.findall('link')}
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    assert len(children) == len(set(children))
    assert links - set(children) == {'world'}


def test_meshes_exist():
    robot = load_urdf()
    prefix = 'package://braco_description/'
    meshes = {m.get('filename') for m in robot.iter('mesh')}
    assert meshes
    for filename in meshes:
        assert filename.startswith(prefix), filename
        assert os.path.isfile(os.path.join(PACKAGE_DIR, filename[len(prefix):])), filename


def test_mesh_textures_exist():
    # A .dae can point to texture images next to it; RViz aborts if one is missing.
    robot = load_urdf()
    prefix = 'package://braco_description/'
    meshes = {m.get('filename')[len(prefix):] for m in robot.iter('mesh')}
    for mesh in meshes:
        path = os.path.join(PACKAGE_DIR, mesh)
        with open(path, encoding='utf-8') as f:
            images = re.findall(
                r'<init_from>([^<]+\.(?:png|jpe?g|tga|bmp))</init_from>', f.read(), re.I)
        for image in images:
            assert os.path.isfile(os.path.join(os.path.dirname(path), image)), (mesh, image)


def test_mesh_materials_are_bound():
    # A material binding pointing at a missing material makes Gazebo and RViz fall back to a
    # plain default material (the vendor's link4.dae had that).
    robot = load_urdf()
    prefix = 'package://braco_description/'
    for mesh in {m.get('filename')[len(prefix):] for m in robot.iter('mesh')}:
        with open(os.path.join(PACKAGE_DIR, mesh), encoding='utf-8') as f:
            text = f.read()
        materials = set(re.findall(r'<material [^>]*id="([^"]+)"', text))
        for target in re.findall(r'<instance_material [^>]*target="#([^"]+)"', text):
            assert target in materials, (mesh, target)


def test_meshes_are_triangulated():
    # Gazebo loads COLLADA <polygons> as an empty mesh, and an empty collision mesh crashes
    # its physics. RViz does not mind, so only this test catches it.
    robot = load_urdf()
    prefix = 'package://braco_description/'
    for mesh in {m.get('filename')[len(prefix):] for m in robot.iter('mesh')}:
        with open(os.path.join(PACKAGE_DIR, mesh), encoding='utf-8') as f:
            assert '<polygons' not in f.read(), mesh


def test_meshes_are_in_metres():
    # MoveIt's mesh loader (geometric_shapes) ignores the COLLADA <unit>, Gazebo and RViz apply
    # it: a millimetre mesh with <unit meter="0.001"> looks right but collides 1000x too big in
    # MoveIt. So every mesh declares unit 1 and the URDF scales the millimetre ones by 0.001
    # (the scale attribute, which all three apply).
    robot = load_urdf()
    prefix = 'package://braco_description/'
    for mesh in {m.get('filename')[len(prefix):] for m in robot.iter('mesh')}:
        with open(os.path.join(PACKAGE_DIR, mesh), encoding='utf-8') as f:
            unit = re.search(r'<unit [^>]*meter="([^"]+)"', f.read())
        assert unit and float(unit.group(1)) == 1.0, (mesh, unit and unit.group(0))


def test_actuated_gripper_has_one_master_and_five_mimics():
    robot = load_urdf(gripper_actuated='true')
    mimics = robot.findall('.//mimic')
    assert len(mimics) == 5
    assert {m.get('joint') for m in mimics} == {'gripper_controller'}
    assert robot.find("joint[@name='gripper_controller']").get('type') == 'revolute'


def test_gazebo_drives_the_gripper_through_its_master_joint():
    # #123: in Gazebo the gripper moves. Only its master, gripper_controller, takes a command;
    # the five mimic joints report their state, moved by the physics engine, which must be one
    # that enforces <mimic> (Bullet Featherstone; DART does not).
    robot = load_urdf(gripper_actuated='true', gazebo='true')
    joints = robot.find('ros2_control').findall('joint')
    commanded = [j.get('name') for j in joints if j.find('command_interface') is not None]
    assert commanded == ARM_JOINTS + ['gripper_controller']
    mimics = {j.get('name') for j in robot.findall('joint') if j.find('mimic') is not None}
    assert {j.get('name') for j in joints} - set(commanded) == mimics
    world = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'lab_bench.sdf')).getroot()
    engine = world.find(".//plugin[@name='gz::sim::systems::Physics']/engine/filename")
    assert engine is not None and 'bullet-featherstone' in engine.text


def test_gripper_action_controller_drives_the_master_joint():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PACKAGE_DIR, 'config', 'controllers.yaml'), encoding='utf-8') as f:
        config = yaml.safe_load(f)
    manager = config['controller_manager']['ros__parameters']
    assert manager['gripper_action_controller']['type'] == \
        'parallel_gripper_action_controller/GripperActionController'
    assert config['gripper_action_controller']['ros__parameters']['joint'] == 'gripper_controller'


def test_gazebo_wrist_camera_matches_the_real_camera():
    # #123: Gazebo renders the wrist camera at camera_link, with the real camera's size and rate
    # (braco_bringup/config/braco.yaml) and its images in camera_optical_frame, like the real
    # ones. The world must load the Sensors system, or no camera renders.
    yaml = pytest.importorskip('yaml')
    robot = load_urdf(gazebo='true')
    sensors = [(g.get('reference'), s) for g in robot.findall('gazebo')
               for s in g.findall("sensor[@type='camera']")]
    assert len(sensors) == 1
    reference, sensor = sensors[0]
    assert reference == 'camera_link'
    assert sensor.find('gz_frame_id').text == 'camera_optical_frame'
    assert sensor.find('topic').text == '/camera/image_raw'
    bringup = os.path.join(PACKAGE_DIR, '..', 'braco_bringup', 'config', 'braco.yaml')
    with open(bringup, encoding='utf-8') as f:
        real = yaml.safe_load(f)['camera']['ros__parameters']
    image = sensor.find('camera/image')
    assert int(image.find('width').text) == real['width']
    assert int(image.find('height').text) == real['height']
    assert float(sensor.find('update_rate').text) == real['fps']
    assert load_urdf().find('.//sensor') is None   # the real arm's model has no sensor
    world = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'lab_bench.sdf')).getroot()
    assert world.find(".//plugin[@name='gz::sim::systems::Sensors']") is not None


def test_fixed_gripper_has_no_movable_finger_joints():
    robot = load_urdf(gripper_actuated='false')
    assert robot.findall('.//mimic') == []
    movable = [j.get('name') for j in robot.findall('joint') if j.get('type') != 'fixed']
    assert movable == ARM_JOINTS


def rotation(r, p, y):
    """URDF rpy (fixed axes x, y, z) as a 3x3 matrix: Rz(y) * Ry(p) * Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def test_zero_pose_tool0_position_is_unchanged():
    """Check the forward kinematics at all joints = 0, down to tool0."""
    # All joint angles are zero, so only the fixed origins of the chain count.
    robot = load_urdf()
    parent_of = {j.find('child').get('link'): j for j in robot.findall('joint')}
    chain, link = [], 'tool0'
    while link in parent_of:
        chain.append(parent_of[link])
        link = parent_of[link].find('parent').get('link')
    rot = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pos = [0.0, 0.0, 0.0]
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = floats(origin.get('xyz'))
        rpy = floats(origin.get('rpy'))
        pos = [pos[i] + sum(rot[i][k] * xyz[k] for k in range(3)) for i in range(3)]
        local = rotation(*rpy)
        rot = [[sum(rot[i][k] * local[k][j] for k in range(3)) for j in range(3)]
               for i in range(3)]
    for got, want in zip(pos, ZERO_TOOL0):
        assert got == pytest.approx(want, abs=1e-3)


def test_pedestal_look_does_not_change_the_mount():
    # The bench's pedestal is only the simulated look of g_base: the height of the arm (the
    # table-clearance check depends on it) and the collision box are the same in both models.
    plain, gazebo = load_urdf(), load_urdf(gazebo='true')
    assert plain.find("link[@name='g_base']/visual/geometry/mesh") is not None
    names = [v.get('name') for v in gazebo.findall("link[@name='g_base']/visual")]
    assert names == ['pedestal']
    assert ET.tostring(plain.find("link[@name='g_base']/collision")) == \
        ET.tostring(gazebo.find("link[@name='g_base']/collision"))
    joint = "joint[@name='g_base_to_base_link']/origin"
    assert plain.find(joint).get('xyz') == gazebo.find(joint).get('xyz')


def test_arm_spawns_on_the_lab_bench_table():
    # The model's world frame is the table surface: robot_z must be the height of the table top,
    # and the default robot_x, robot_y must put the mount on one table, flush with its rear edge.
    # The tables are turned 90 deg in the world: their long side runs along x, along the gap.
    with open(os.path.join(PACKAGE_DIR, 'launch', 'gazebo.launch.py'), encoding='utf-8') as f:
        launch = f.read()

    def default(name):
        match = re.search(r"'%s', default_value='([^']+)'" % name, launch)
        assert match, name
        return float(match.group(1))

    table = ET.parse(os.path.join(PACKAGE_DIR, 'models', 'lab_table', 'model.sdf')).getroot()
    top = table.find(".//collision[@name='top_collision']")
    short_side, long_side, thickness = floats(top.find('geometry/box/size').text)
    assert floats(top.find('pose').text)[2] + thickness / 2 == pytest.approx(default('robot_z'))

    world = ET.parse(os.path.join(PACKAGE_DIR, 'worlds', 'lab_bench.sdf')).getroot()
    poses = {i.find('name').text: floats(i.find('pose').text) for i in world.iter('include')}
    centre = poses['lab_table_left']
    assert centre[5] == pytest.approx(math.pi / 2, abs=1e-3)
    assert poses['lab_table_right'][5] == pytest.approx(math.pi / 2, abs=1e-3)
    pedestal = load_urdf(gazebo='true').find(
        "link[@name='g_base']/visual[@name='pedestal']/geometry/box")
    pedestal_depth, pedestal_width, _ = floats(pedestal.get('size'))
    assert default('robot_x') - pedestal_depth / 2 == pytest.approx(centre[0] - long_side / 2)
    assert default('robot_y') - pedestal_width / 2 >= centre[1] - short_side / 2
    assert default('robot_y') + pedestal_width / 2 <= centre[1] + short_side / 2


def test_plain_model_has_no_ros2_control():
    assert load_urdf().find('ros2_control') is None


def test_gazebo_model_controls_the_arm_joints():
    robot = load_urdf(gazebo='true')
    control = robot.find('ros2_control')
    assert control is not None
    assert [j.get('name') for j in control.findall('joint')] == ARM_JOINTS
    params = robot.find('gazebo/plugin/parameters').text
    assert os.path.isfile(params), params


def test_controllers_drive_the_arm_joints():
    # config/controllers.yaml is the one set for the simulation and the real arm: arm_controller
    # (the name MoveIt and the web page use) commands exactly the joints of <ros2_control>.
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PACKAGE_DIR, 'config', 'controllers.yaml'), encoding='utf-8') as f:
        config = yaml.safe_load(f)
    manager = config['controller_manager']['ros__parameters']
    assert manager['arm_controller']['type'] == \
        'joint_trajectory_controller/JointTrajectoryController'
    assert 'joint_state_broadcaster' in manager
    arm = config['arm_controller']['ros__parameters']
    assert arm['joints'] == ARM_JOINTS
    assert arm['command_interfaces'] == ['position']


def test_arm_controller_fails_a_goal_it_does_not_reach():
    # #123: without goal tolerances a goal blocked by the table (0.33 rad short) ended as
    # successful. Every joint needs a goal tolerance above the real servos' dead zone (~3 deg)
    # and below that, a path tolerance above Gazebo's lag (0.047 rad), and goal_time > 0, or a
    # blocked goal never ends.
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(PACKAGE_DIR, 'config', 'controllers.yaml'), encoding='utf-8') as f:
        constraints = yaml.safe_load(f)['arm_controller']['ros__parameters']['constraints']
    assert constraints['goal_time'] > 0
    for joint in ARM_JOINTS:
        assert math.radians(3) < constraints[joint]['goal'] < 0.33, joint
        assert constraints[joint]['trajectory'] > 0.047, joint
