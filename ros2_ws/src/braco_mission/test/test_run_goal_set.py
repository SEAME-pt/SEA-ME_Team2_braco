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

"""The goal set of scripts/run_goal_set.py and its summary: no ROS graph, no arm."""

import copy
import csv
import importlib.util
import math
import os
from types import SimpleNamespace

import pytest
import yaml

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PACKAGE, 'scripts', 'run_goal_set.py')
GOAL_SET = os.path.join(PACKAGE, 'config', 'goal_set.yaml')
_spec = importlib.util.spec_from_file_location('run_goal_set', SCRIPT)
run_goal_set = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(run_goal_set)

START = {'position': [0.2167, -0.0646, 0.1126], 'orientation': [0.9238, -0.3826, -0.0135, 0.0056]}
VALID = {
    'frame': 'world',
    'start': START,
    'goals': [
        {'name': 'line_y', 'line': [0.2167, 0.0654, 0.1126]},
        {'name': 'pose_a', 'pose': START},
    ],
}


def with_goal(goal):
    data = copy.deepcopy(VALID)
    data['goals'].append(goal)
    return data


def test_the_package_goal_set_is_valid():
    with open(GOAL_SET) as f:
        goal_set = run_goal_set.parse_goal_set(yaml.safe_load(f))
    assert goal_set['frame'] == 'world'
    kinds = {g['kind'] for g in goal_set['goals']}
    assert kinds == {'line', 'pose'}   # both actions of #140


def test_parse_valid_set():
    goal_set = run_goal_set.parse_goal_set(VALID)
    assert goal_set['start']['position'] == (0.2167, -0.0646, 0.1126)
    assert [g['kind'] for g in goal_set['goals']] == ['line', 'pose']
    assert goal_set['goals'][0]['line'] == (0.2167, 0.0654, 0.1126)


@pytest.mark.parametrize('goal', [
    {'name': 'line_y', 'line': [0.1, 0.0, 0.1]},                      # name used twice
    {'name': 'start', 'line': [0.1, 0.0, 0.1]},                       # reserved name
    {'line': [0.1, 0.0, 0.1]},                                        # no name
    {'name': 'both', 'line': [0.1, 0.0, 0.1], 'pose': START},         # line and pose
    {'name': 'neither'},
    {'name': 'short', 'line': [0.1, 0.0]},
    {'name': 'nan', 'line': [0.1, float('nan'), 0.1]},
    {'name': 'text', 'line': [0.1, 'a', 0.1]},
    {'name': 'not_unit', 'pose': {'position': [0.1, 0.0, 0.1], 'orientation': [0, 0, 0, 2]}},
    {'name': 'no_orient', 'pose': {'position': [0.1, 0.0, 0.1]}},
    {'name': 'typo', 'lien': [0.1, 0.0, 0.1]},
])
def test_parse_refuses(goal):
    with pytest.raises(ValueError):
        run_goal_set.parse_goal_set(with_goal(goal))


def test_parse_refuses_no_start_and_no_goals():
    with pytest.raises(ValueError):
        run_goal_set.parse_goal_set({'goals': VALID['goals']})
    with pytest.raises(ValueError):
        run_goal_set.parse_goal_set({'start': START, 'goals': []})
    with pytest.raises(ValueError):
        run_goal_set.parse_goal_set(None)


def test_select():
    goals = run_goal_set.parse_goal_set(VALID)['goals']
    assert run_goal_set.select(goals, '') == goals
    assert [g['name'] for g in run_goal_set.select(goals, 'pose_a, line_y')] == \
        ['line_y', 'pose_a']   # the set's order
    with pytest.raises(ValueError):
        run_goal_set.select(goals, 'line_y,nope')


def line_result(**kw):
    values = {'success': True, 'fraction': 1.0, 'max_deviation_m': 0.00004,
              'position_error_m': 0.0003, 'orientation_error_rad': math.radians(0.08),
              'message': 'reached: position error 0.3 mm'}
    values.update(kw)
    return SimpleNamespace(**values)


def test_summary_row_line():
    executed = {'max_deviation_m': 0.000327, 'mean_deviation_m': 0.00023,
                'end_error_m': 0.00035}
    row = run_goal_set.summary_row('line_y', 'line', line_result(), executed)
    assert row['success'] is True
    assert row['position_error_mm'] == '0.300'
    assert row['orientation_error_deg'] == '0.080'
    assert row['fraction'] == '1.000'
    assert row['planned_deviation_mm'] == '0.040'
    assert row['executed_deviation_max_mm'] == '0.327'
    assert 'line_y' in run_goal_set.format_row(row)


def test_summary_row_pose_position_only():
    result = SimpleNamespace(success=True, position_error_m=0.0001,
                             orientation_error_rad=float('nan'), message='reached')
    row = run_goal_set.summary_row('pose_a', 'pose', result)
    assert row['orientation_error_deg'] == ''
    assert row['fraction'] == ''
    assert 'FAIL' not in run_goal_set.format_row(row)


def test_moved():
    assert run_goal_set.moved(line_result())
    assert run_goal_set.moved(line_result(success=False,
                                          message='outside the tolerance: position error 7 mm'))
    assert not run_goal_set.moved(line_result(success=False,
                                              message='only 54.5% of the path planned'))


def test_write_summary(tmp_path):
    rows = [run_goal_set.summary_row('line_y', 'line', line_result())]
    path = tmp_path / 'summary.csv'
    run_goal_set.write_summary(str(path), rows)
    with open(path) as f:
        read = list(csv.DictReader(f))
    assert read[0]['name'] == 'line_y'
    assert list(read[0]) == run_goal_set.SUMMARY_FIELDS


def test_too_hot():
    assert run_goal_set.too_hot(None, 55) is None            # simulation: nothing read
    assert run_goal_set.too_hot([42, 38, 35, 54, 35, 37], 55) is None
    why = run_goal_set.too_hot([42, 38, 35, 57, 35, 55], 55)
    assert 'J4 57 C' in why and 'J6 55 C' in why and 'J1' not in why


def test_hint():
    assert 'make zero' in run_goal_set.hint('no plan: no plan to the pose')
    assert 'servos' in run_goal_set.hint(
        'execution failed: arm_controller did not complete the trajectory')
    assert run_goal_set.hint('control refused: owner web') is None


def test_summary_row_keeps_the_temperature():
    row = run_goal_set.summary_row('line_y', 'line', line_result(), temp=48)
    assert row['max_servo_temp_c'] == 48
    assert run_goal_set.summary_row('start', 'pose', line_result())['max_servo_temp_c'] == ''
