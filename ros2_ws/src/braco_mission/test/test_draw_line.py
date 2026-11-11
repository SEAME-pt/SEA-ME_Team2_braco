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

"""Path geometry and outputs of scripts/draw_line.py: no ROS graph, no arm."""

import csv
import importlib.util
import os

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      'scripts', 'draw_line.py')
_spec = importlib.util.spec_from_file_location('draw_line', SCRIPT)
draw_line = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(draw_line)

A = (0.2, -0.05, 0.1)
B = (0.2, 0.05, 0.1)   # 10 cm along y


def test_distance_to_segment():
    assert draw_line.distance_to_segment((0.2, 0.0, 0.1), A, B) == pytest.approx(0.0)
    assert draw_line.distance_to_segment((0.202, 0.0, 0.1), A, B) == pytest.approx(0.002)
    assert draw_line.distance_to_segment((0.2, 0.08, 0.14), A, B) == pytest.approx(0.05)
    assert draw_line.distance_to_segment((0.2, -0.08, 0.06), A, B) == pytest.approx(0.05)
    assert draw_line.distance_to_segment((0.23, -0.05, 0.14), A, A) == pytest.approx(0.05)


def test_along_is_clamped_to_the_line():
    assert draw_line.along((0.21, 0.0, 0.1), A, B) == pytest.approx(0.05)
    assert draw_line.along((0.2, -0.2, 0.1), A, B) == pytest.approx(0.0)
    assert draw_line.along((0.2, 0.2, 0.1), A, B) == pytest.approx(0.1)
    assert draw_line.along((0.3, 0.0, 0.1), A, A) == 0.0


def test_too_short():
    assert draw_line.too_short(A, B) is None
    assert 'already 0.5 mm from the end' in draw_line.too_short(A, (0.2, -0.0495, 0.1))
    assert draw_line.too_short(A, A) is not None


def test_analyse_and_summary():
    samples = [(0.0, 0.2, -0.05, 0.1), (1.0, 0.201, 0.0, 0.1), (2.0, 0.2, 0.049, 0.1)]
    rows = draw_line.analyse(samples, A, B)
    assert [r[0] for r in rows] == [0.0, 1.0, 2.0]
    assert rows[1][4] == pytest.approx(0.05)
    assert rows[1][5] == pytest.approx(0.001)
    s = draw_line.summary(rows, A, B)
    assert s['length_m'] == pytest.approx(0.1)
    assert s['max_deviation_m'] == pytest.approx(0.001)
    assert s['mean_deviation_m'] == pytest.approx(0.001 / 3)
    assert s['end_error_m'] == pytest.approx(0.001)
    assert s['samples'] == 3
    assert draw_line.summary([], A, B) is None


def test_write_csv(tmp_path):
    rows = draw_line.analyse([(0.0, 0.2, -0.05, 0.1), (0.5, 0.2, 0.05, 0.1)], A, B)
    path = tmp_path / 'line.csv'
    draw_line.write_csv(str(path), rows)
    with open(path) as f:
        table = list(csv.reader(f))
    assert table[0] == ['t_s', 'x_m', 'y_m', 'z_m', 'along_m', 'deviation_m']
    assert table[2] == ['0.5000', '0.200000', '0.050000', '0.100000', '0.100000', '0.000000']


def test_plot_writes_a_png(tmp_path):
    pytest.importorskip('matplotlib')
    rows = draw_line.analyse([(0.0, 0.2, -0.05, 0.1), (0.5, 0.2, 0.05, 0.1)], A, B)
    path = tmp_path / 'line.png'
    draw_line.plot(str(path), rows, A, B, 'test')
    with open(path, 'rb') as f:
        assert f.read(8) == b'\x89PNG\r\n\x1a\n'
