"""Unit tests of coverage_touch.py: the ROS Python files no test ran are added at 0%.

Run by make coverage, in the ci container (the coverage.py the helper uses there). The helper runs
as mk/ci.mk calls it, a subprocess with a data file, a root and the relative paths.
/scripts is mounted read-only there: run with python3 -B and -p no:cacheprovider.
"""

import subprocess
import sys
from pathlib import Path

from coverage import CoverageData

HELPER = Path(__file__).with_name('coverage_touch.py')


def touch(data_file, root, *files):
    return subprocess.run(
        [sys.executable, str(HELPER), '--data-file', str(data_file), '--root', str(root), *files],
        capture_output=True,
        text=True,
    )


def measured(tmp_path, lines):
    """Write a data file where each file of `lines` (relative to tmp_path) ran those lines."""
    data = CoverageData(basename=str(tmp_path / '.coverage'))
    data.add_lines({str(tmp_path / f): ls for f, ls in lines.items()})
    data.write()
    return tmp_path / '.coverage'


def read(data_file):
    data = CoverageData(basename=str(data_file))
    data.read()
    return data


def test_adds_the_unmeasured_file_at_0_percent(tmp_path):
    data_file = measured(tmp_path, {'pkg/launch/a.launch.py': [1, 2, 3]})
    result = touch(data_file, tmp_path, 'pkg/launch/a.launch.py', 'pkg/launch/b.launch.py')
    assert result.returncode == 0, result.stderr
    assert 'coverage_touch: 1 of 2 file(s) added at 0%' in result.stdout
    data = read(data_file)
    assert data.measured_files() == {
        str(tmp_path / 'pkg/launch/a.launch.py'),
        str(tmp_path / 'pkg/launch/b.launch.py'),
    }
    assert data.lines(str(tmp_path / 'pkg/launch/b.launch.py')) == []


def test_keeps_the_lines_of_a_measured_file(tmp_path):
    data_file = measured(tmp_path, {'pkg/launch/a.launch.py': [1, 2, 3]})
    result = touch(data_file, tmp_path, 'pkg/launch/a.launch.py')
    assert result.returncode == 0, result.stderr
    assert 'coverage_touch: 0 of 1 file(s) added at 0%' in result.stdout
    assert sorted(read(data_file).lines(str(tmp_path / 'pkg/launch/a.launch.py'))) == [1, 2, 3]


def test_no_measured_file_adds_all_at_0_percent(tmp_path):
    """Pytest ran but measured nothing: every file at 0%, so the floor fails, not this helper."""
    data_file = measured(tmp_path, {})
    result = touch(data_file, tmp_path, 'pkg/launch/a.launch.py', 'pkg/launch/b.launch.py')
    assert result.returncode == 0, result.stderr
    assert 'coverage_touch: 2 of 2 file(s) added at 0%' in result.stdout


def test_missing_data_file_fails(tmp_path):
    """No pytest data at all: coverage.py raises DataError, so mk/ci.mk fails the target."""
    assert touch(tmp_path / '.coverage', tmp_path, 'pkg/launch/a.launch.py').returncode != 0
