#!/usr/bin/env python3
"""Add the ROS Python files no test ran to a combined coverage.py data file, at 0%.

pytest-cov only records the files a test imported: a launch file no test loads is missing from
the report, so the total never drops. coverage.py's own [run] source walker cannot fill the gap:
it skips `*.launch.py` and directories without `__init__.py`. Given the in-scope files, this marks
each one as measured; a file already measured keeps its lines. No filtering here: the caller
passes the list (`git ls-files` in mk/ci.mk). Empty data (no pytest ran) raises DataError, so the
caller fails. Needs coverage.py >= 5, Python 3.8+.

    coverage_touch.py --data-file coverage/.coverage --root /ws/src \
        braco_bringup/launch/braco.launch.py ...
"""

from __future__ import annotations

import argparse
from pathlib import Path

from coverage import CoverageData


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--data-file', required=True, help='the combined .coverage file')
    parser.add_argument('--root', required=True, type=Path, help='the paths are relative to it')
    parser.add_argument('files', nargs='+', help='the in-scope .py files, relative to --root')
    args = parser.parse_args()

    data = CoverageData(basename=args.data_file)
    data.read()
    measured = set(data.measured_files())
    files = [str(args.root.resolve() / f) for f in args.files]
    data.touch_files(files)
    data.write()
    added = len(set(files) - measured)
    print(f'coverage_touch: {added} of {len(files)} file(s) added at 0%')


if __name__ == '__main__':
    main()
