#!/usr/bin/env python3
"""Print a markdown coverage summary for the GitHub Actions job summary ($GITHUB_STEP_SUMMARY).

Reads gcovr's --json-summary (C++, one row per package) and coverage.py's `coverage json` (Python), and marks each
gated total against its floor. A missing file is reported as "no data", not as an error: the report it comes from
may not have been written (a failed build, or nothing left to measure). Standard library only, Python 3.8+.

    coverage_summary.py --gcovr "C++ (ros2_ws)" cxx.json 80 --coverage-py "Web (web/server)" coverage.json 80
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path


def pct(covered: int, total: int) -> str:
    return f"{100.0 * covered / total:.1f}% ({covered}/{total})" if total else "-"


def status(value: float, floor: float) -> str:
    return "✅" if value >= floor else "❌"


def gcovr_rows(data: dict, floor: float) -> list[str]:
    """The gated line total first, then one row per package (the first path component under the gcovr root)."""
    rows = [
        f"| **total** | **{pct(data['line_covered'], data['line_total'])}** "
        f"| {pct(data['branch_covered'], data['branch_total'])} "
        f"| {floor:g}% of lines | {status(data['line_percent'], floor)} |"
    ]
    packages: dict[str, list[int]] = defaultdict(lambda: [0, 0, 0, 0])
    for f in data["files"]:
        p = packages[f["filename"].split("/", 1)[0]]
        p[0] += f["line_covered"]
        p[1] += f["line_total"]
        p[2] += f["branch_covered"]
        p[3] += f["branch_total"]
    for name, (lc, lt, bc, bt) in sorted(packages.items()):
        rows.append(f"| `{name}` | {pct(lc, lt)} | {pct(bc, bt)} | | |")
    return rows


def coverage_py_rows(data: dict, floor: float) -> list[str]:
    """coverage.py gates on statements and branches together (percent_covered), not on the line percentage."""
    t = data["totals"]
    gated = t["percent_covered"]
    return [
        f"| **total** | **{pct(t['covered_lines'], t['num_statements'])}** "
        f"| {pct(t['covered_branches'], t['num_branches'])} "
        f"| {floor:g}% of statements + branches ({gated:.1f}%) | {status(gated, floor)} |"
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for flag in ("--gcovr", "--coverage-py"):
        parser.add_argument(flag, nargs=3, action="append", default=[], metavar=("NAME", "JSON", "FLOOR"))
    args = parser.parse_args()

    sections = [(s, gcovr_rows) for s in args.gcovr] + [(s, coverage_py_rows) for s in args.coverage_py]
    for (name, path, floor), rows in sections:
        print(f"### Coverage: {name}\n")
        file = Path(path)
        if not file.is_file():
            print("No data (no report was written, or nothing to measure).\n")
            continue
        print("| | Lines | Branches | Floor | |\n|---|---|---|---|---|")
        print("\n".join(rows(json.loads(file.read_text()), float(floor))), end="\n\n")


if __name__ == "__main__":
    main()
