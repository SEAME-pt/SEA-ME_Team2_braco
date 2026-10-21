#!/usr/bin/env bash
# restart.sh - restart only the braco web page inside the running arm container.
#
# The arm container starts the page from arm_routine.sh and does not respawn it, so after a
# change in web/ this replaces the uvicorn process without touching the driver or the arm
# (no zero, no park). Run it with `make arm-web-restart` from the repo root.
#
# The new server logs to the container's output, so `make arm-web-logs` keeps working.
# Presence and the control lease live in the server's memory: everybody reconnects by
# themselves and has to take control again.
set -u

WEB_DIR=${WEB_DIR:-/opt/braco}
WEB_PORT=${WEB_PORT:-8000}
PATTERN="uvicorn web.server.app:app"

# python3 instead of pkill/pgrep: the image is not guaranteed to have procps
pids() {
    python3 - "$PATTERN" <<'PY'
import os, sys
pattern, me = sys.argv[1], os.getpid()
for pid in filter(str.isdigit, os.listdir("/proc")):
    try:
        cmd = open("/proc/%s/cmdline" % pid).read().replace("\0", " ")
    except OSError:
        continue
    if pattern in cmd and int(pid) != me and "restart.sh" not in cmd:
        print(pid)
PY
}

old=$(pids)
if [ -n "$old" ]; then
    kill -TERM $old 2>/dev/null
    for _ in 1 2 3 4 5; do [ -z "$(pids)" ] && break; sleep 1; done
    [ -n "$(pids)" ] && kill -KILL $(pids) 2>/dev/null
    echo "web: old server stopped"
fi

if [ ! -f "$WEB_DIR/web/server/app.py" ]; then
    echo "web: $WEB_DIR/web is not mounted, nothing to start" >&2
    exit 1
fi

cd "$WEB_DIR" || exit 1
ARM_SOURCE=${ARM_SOURCE:-ros} nohup setsid python3 -m uvicorn web.server.app:app \
    --host 0.0.0.0 --port "$WEB_PORT" --workers 1 --no-access-log \
    > /proc/1/fd/1 2>&1 < /dev/null &

sleep 3
if [ -n "$(pids)" ]; then
    echo "web: restarted on port $WEB_PORT"
else
    echo "web: failed to start, see make arm-logs" >&2
    exit 1
fi
