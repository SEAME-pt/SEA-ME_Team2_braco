#!/usr/bin/env bash
# Fails when a document references an issue or PR by number (docs/guides/docs-rules.md, section 11).
#
# Usage: scripts/check-doc-refs.sh [file...]
# Without arguments it checks every tracked .md file except the GitHub issue templates.
# Prints file:line:text for each hit and exits 1 if there is any, 0 otherwise.
#
# `#12`, `PR #12`, `issues/12` and `pull/12` are hits, also inside code spans. Section anchors such as
# `#1-set-up` are not: a number followed by a word character or `-` is part of an anchor.

set -euo pipefail

PATTERN='(?<![\w/])#\d+(?![\w-])|issues/\d+|pull/\d+'

if [ "$#" -eq 0 ]; then
    cd "$(git rev-parse --show-toplevel)"
    mapfile -t files < <(git ls-files '*.md' ':!.github/ISSUE_TEMPLATE/')
else
    files=("$@")
fi

# grep exits 0 on a hit, 1 on none, 2 on an error (e.g. a missing file).
status=0
grep -nHP "$PATTERN" -- "${files[@]}" || status=$?

case $status in
    0) echo "Issue or PR references found: describe what they stand for instead (docs/guides/docs-rules.md)." >&2
       exit 1 ;;
    1) exit 0 ;;
    *) exit "$status" ;;
esac
