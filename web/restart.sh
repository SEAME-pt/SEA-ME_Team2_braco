#!/usr/bin/env bash
# restart.sh - start (or restart) the braco web page inside a running container; `stop` stops it.
#
# `make web` runs it in the arm container (`real`, Jetson) or in `dev` (PC), `make web-stop` with `stop`.
# The page is not part of `make up`: the containers only run the ROS 2 nodes, so replacing the page
# never touches the driver or the arm (no zero, no park). By hand, from the repo root:
#   docker compose --project-directory docker exec real ros_entrypoint.sh bash /opt/braco/web/restart.sh
#
# The server logs to the container's output, so `make logs S=web` shows it.
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
if [ "${1:-}" = stop ]; then
    [ -n "$old" ] || echo "web: not running"
    exit 0
fi

if [ ! -f "$WEB_DIR/web/server/app.py" ]; then
    echo "web: $WEB_DIR/web is not mounted, nothing to start" >&2
    exit 1
fi

cd "$WEB_DIR" || exit 1
# Into the container's output (make logs). Not nohup: when that output is a terminal (dev has tty: true) nohup
# sends everything to ~/nohup.out instead; ignoring SIGHUP is the part of nohup the page needs.
(trap '' HUP; ARM_SOURCE=${ARM_SOURCE:-ros} exec setsid python3 -m uvicorn web.server.app:app \
    --host 0.0.0.0 --port "$WEB_PORT" --workers 1 --no-access-log) \
    > /proc/1/fd/1 2>&1 < /dev/null &

sleep 3
if [ -n "$(pids)" ]; then
    if [ "${ARM_MOCK:-}" = 1 ]; then echo "web: started on port $WEB_PORT (mock arm data)"; else echo "web: started on port $WEB_PORT (ARM_SOURCE=${ARM_SOURCE:-ros})"; fi
else
    echo "web: failed to start, see make logs" >&2
    exit 1
fi
