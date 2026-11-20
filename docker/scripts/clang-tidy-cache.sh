#!/bin/sh
# clang-tidy through ctcache, with clang-tidy's own arguments: make tidy passes it to run-clang-tidy as
# -clang-tidy-binary. The cache directory and options come from the environment (CTCACHE_DIR, set by make tidy).
exec python3 /usr/local/lib/ctcache/clang_tidy_cache.py /usr/bin/clang-tidy "$@"
