#!/bin/sh
# Talker (C++) and listener (Python) in one container.
# PASS iff the listener logs "I heard" within SMOKE_TIMEOUT seconds.
# Success is judged by output, not by exit codes (timeout/kill return non-zero).
export PYTHONUNBUFFERED=1
TIMEOUT="${SMOKE_TIMEOUT:-20}"
LOG=$(mktemp)

ros2 run demo_nodes_cpp talker >/dev/null 2>&1 &
TALKER=$!
ros2 run demo_nodes_py listener >"$LOG" 2>&1 &
LISTENER=$!

i=0
ok=1
while [ "$i" -lt "$TIMEOUT" ]; do
	if grep -q "I heard" "$LOG"; then
		ok=0
		break
	fi
	i=$((i + 1))
	sleep 1
done

kill "$TALKER" "$LISTENER" 2>/dev/null
wait 2>/dev/null

if [ "$ok" -eq 0 ]; then
	echo "PASS: listener received messages"
else
	echo "FAIL: no 'I heard' within ${TIMEOUT}s"
	cat "$LOG"
fi
exit "$ok"
