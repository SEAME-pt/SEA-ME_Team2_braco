#!/usr/bin/env bash
# Bench test of issue #55 (docs/decisions/ADR-0002-unique-control.md, docs/decisions/ADR-0003-rest-per-temp.md).
#
# Run it with `make bench` on the Jetson with the system up and the arm powered. Checks that can be read from
# ROS (owner, rest mode) are automatic; the ones that need eyes or hands (the arm moving, the stick, killing the
# page) ask the operator. The result is printed and appended to bench-report.log.
#
# The arm does not move on its own: motion commands are only sent when the operator confirms the area is clear.
#
# BENCH_EXEC  command prefix that runs a command inside the system container (set by `make bench`)

set -u

EXEC=${BENCH_EXEC:?run it with: make bench}
REPORT=${BENCH_REPORT:-bench-report.log}
IDLE_S=${BENCH_IDLE_S:-5}
HOT_C=${BENCH_HOT_C:-66}
STOP_HOLD_S=${BENCH_STOP_HOLD_S:-3.5}   # arm_driver stop_hold_s (3 s) plus a margin

pass=0
fail=0

ros() { $EXEC ros2 "$@"; }

say() { printf '\n== %s\n' "$*"; }

record() {  # record PASS|FAIL|SKIP "what"
  printf '%-4s %s\n' "$1" "$2" | tee -a "$REPORT"
  case $1 in PASS) pass=$((pass + 1)) ;; FAIL) fail=$((fail + 1)) ;; esac
}

ask() {  # ask "question": y = yes
  local a
  read -r -p "$1 [y/N] " a
  [[ $a == y || $a == Y ]]
}

owner() {
  ros topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /arm/control_owner std_msgs/msg/String 2>/dev/null | sed -n 's/^data: *//p' | tr -d "'\r"
}

read_only() {
  ros topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /arm/read_only std_msgs/msg/Bool 2>/dev/null | sed -n 's/^data: *//p' | tr -d '\r'   # YAML booleans: true / false
}

take() {  # take SOURCE [force|release]
  local force=false release=false
  [[ ${2:-} == force ]] && force=true
  [[ ${2:-} == release ]] && release=true
  ros service call /arm/take_control braco_interfaces/srv/TakeControl \
    "{source: '$1', force: $force, release: $release}" 2>&1 | tr -d '\r'
}

expect_owner() {  # expect_owner WANTED "what"
  local got
  got=$(owner)
  if [[ $got == "$1" ]]; then record PASS "$2 (owner '$got')"; else record FAIL "$2 (owner '$got', wanted '$1')"; fi
}

cleanup() {
  # HALT, not STOP: the bench ending is no emergency and must not leave the arm latched
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 3, source: 'script'}" >/dev/null 2>&1
  take script release >/dev/null 2>&1
  ros service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  [[ -n ${FAKE_PID:-} ]] && kill "$FAKE_PID" 2>/dev/null
  stop_stream
  [[ $(stopped) == true ]] && echo "bench: STOP is still latched: make resume" >&2
}

stop_stream() {  # killing docker exec leaves the publisher running in the container
  $EXEC pkill -f 'topic pub -r 10 /arm/command' 2>/dev/null
  [[ -n ${STREAM_PID:-} ]] && kill "$STREAM_PID" 2>/dev/null
  [[ -n ${KEEPALIVE_PID:-} ]] && kill "$KEEPALIVE_PID" 2>/dev/null
  unset STREAM_PID KEEPALIVE_PID
}
trap cleanup EXIT

wait_publisher() {  # [t] keeps pgrep from matching its own command line
  until $EXEC pgrep -f '[t]opic pub -r 10 /arm/command' >/dev/null 2>&1; do sleep 0.5; done
}

stopped() {
  ros topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /arm/stopped std_msgs/msg/Bool 2>/dev/null | sed -n 's/^data: *//p' | tr -d '\r'
}

printf '\n# bench %s on %s\n' "$(date -Iseconds)" "$(hostname)" >>"$REPORT"
ros service list 2>/dev/null | grep -q /arm/take_control || { echo "arm_driver is not running: make up" >&2; exit 1; }

if [[ $(stopped) == true ]]; then echo "STOP is latched: make resume first" >&2; exit 1; fi
if [[ $(read_only) == true ]]; then echo "rest mode is on: make resume first (then make zero)" >&2; exit 1; fi

say "1. Control: free, taken, refused, forced"
take script release >/dev/null
expect_owner "" "the control starts free"
# Each CLI call needs ~5 s to start on the Nano, as long as control_idle_s: an empty JOINTS command from the
# owner moves nothing but keeps the lease alive (the page's stream does the same while it moves)
KEEPALIVE_PID=
$EXEC timeout 60 ros2 topic pub -r 10 /arm/command braco_interfaces/msg/ArmCommand \
  "{type: 1, source: 'web'}" >/dev/null 2>&1 &
KEEPALIVE_PID=$!
wait_publisher
# Read the owner from each answer: a second CLI call would be slower than the lease
out=$(take web)
if grep -q "owner='web'" <<<"$out"; then record PASS "the page takes the control"; else record FAIL "the page did not take the control: $out"; fi
out=$(take joystick)
if grep -q "success=False" <<<"$out" && grep -q "owner='web'" <<<"$out"; then
  record PASS "the joystick is refused while the page owns the control (stick ignored)"
else
  record FAIL "the joystick was not refused: $out"
fi
out=$(take joystick force)
if grep -q "owner='joystick'" <<<"$out"; then record PASS "force takes the control from the page"; else record FAIL "force did not take the control: $out"; fi
stop_stream

say "2. Idle release after ${IDLE_S} s without motion"
take web force >/dev/null
sleep $((IDLE_S + 2))
expect_owner "" "arm_driver frees an idle owner"

say "3. STOP from the terminal while the page streams (#126)"
if ask "Is the arm powered (make zero) and the area clear? Joint 1 will move slowly toward 30 deg"; then
  # Like the page: the same target resent every 0.1 s by the owner. Started first: the CLI needs longer to
  # start on the Nano than control_idle_s, so a control taken before it would expire before the first message.
  $EXEC timeout 60 ros2 topic pub -r 10 /arm/command braco_interfaces/msg/ArmCommand \
    "{type: 1, targets: [{joint: 1, angle_deg: 30.0, speed: 10}], source: 'web'}" >/dev/null 2>&1 &
  STREAM_PID=$!
  wait_publisher
  # The take is itself a slow CLI call, so the stream is publishing (and ignored) by the time it lands
  out=$(take web force)
  if ! grep -q "success=True" <<<"$out"; then
    record FAIL "the page could not take the control for the move: $out"
    exit 1
  fi
  sleep 1
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 0}" >/dev/null
  if ask "Did the arm stop and stay stopped while 'web' keeps streaming (STOP had no source)?"; then
    record PASS "STOP from the terminal holds against a streaming owner"
  else
    record FAIL "STOP from the terminal did not hold against a streaming owner"
  fi
  # The STOP latched (ADR-0005): not even force takes the control, until the release after the hold
  out=$(take joystick force)
  if grep -q "success=False" <<<"$out"; then
    record PASS "STOP latched: force take refused"
  else
    record FAIL "STOP latched: force take was accepted: $out"
  fi
  # The questions above took longer than the hold: a fresh STOP restarts it. The CLI is slow on the
  # Nano, so a release that came after the hold is a SKIP, not a FAIL.
  t0=$(date +%s.%N)
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 0, source: 'web'}" >/dev/null
  out=$(ros service call /arm/release_stop std_srvs/srv/Trigger 2>&1 | tr -d '\r')
  dt=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN {printf "%.1f", b - a}')
  if grep -q "success=False" <<<"$out"; then
    record PASS "release refused inside the hold"
  elif awk -v d="$dt" -v h="$STOP_HOLD_S" 'BEGIN {exit !(d >= h - 0.5)}'; then
    record SKIP "release inside the hold: the CLI took ${dt} s, longer than the hold"
  else
    record FAIL "release inside the hold was accepted after ${dt} s: $out"
  fi
  sleep "$STOP_HOLD_S"
  out=$(ros service call /arm/release_stop std_srvs/srv/Trigger 2>&1 | tr -d '\r')
  if grep -q "success=True" <<<"$out"; then record PASS "release after the hold"; else record FAIL "release after the hold: $out"; fi
  if ask "After the release, with 'web' still streaming, does the arm stay still?"; then
    record PASS "the release moves nothing"
  else
    record FAIL "the arm moved after the release"
  fi
  stop_stream
else
  record SKIP "STOP while the page streams (operator did not confirm the area)"
fi
take web release >/dev/null

say "4. Hand tests"
if ask "Page moving a joint: does the stick do nothing meanwhile?"; then
  record PASS "page moving, stick ignored"
else
  record FAIL "page moving, stick ignored"
fi
# The page owns the control (empty JOINTS keepalive: moves nothing) while the operator holds the button.
# Every owner change is recorded: the CLI is too slow, and the joystick's lease too short, to poll the owner.
OWNERS=$(mktemp)
$EXEC timeout 90 ros2 topic pub -r 10 /arm/command braco_interfaces/msg/ArmCommand \
  "{type: 1, source: 'web'}" >/dev/null 2>&1 &
KEEPALIVE_PID=$!
wait_publisher
$EXEC env PYTHONUNBUFFERED=1 timeout 90 ros2 topic echo --qos-durability transient_local \
  --qos-reliability reliable /arm/control_owner std_msgs/msg/String >"$OWNERS" 2>&1 &
ECHO_PID=$!
out=$(take web force)
if grep -q "success=True" <<<"$out"; then
  echo "The page owns the control. Hold button 8 on the joystick for ~1 s now (waiting up to 60 s)..."
  seen=0
  for _ in $(seq 1 60); do
    if grep -q "data: joystick" "$OWNERS"; then seen=1; break; fi
    sleep 1
  done
  if [[ $seen == 1 ]]; then
    record PASS "hold button: joystick takes over"
  else
    record FAIL "hold button: the joystick never took the control from the page (button 8 = js index 8; run the teleop with debug to see the numbers)"
  fi
else
  record FAIL "hold button: the page could not take the control: $out"
fi
kill "$ECHO_PID" 2>/dev/null
$EXEC pkill -f '[t]opic echo --qos-durability' 2>/dev/null
rm -f "$OWNERS"
stop_stream
take joystick release >/dev/null 2>&1
if ask "Kill the page mid-move (stop the web server): did the watchdog stop the arm (log: 'stopping the arm')?"; then
  record PASS "page killed mid-move: watchdog stops the arm"
else
  record FAIL "page killed mid-move: watchdog stops the arm"
fi

say "5. Simulated HOT servo (${HOT_C} C): the arm parks, releases the joints and rests"
if ask "The arm will fold to the park pose by itself. Is the area clear?"; then
  ros service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}" >/dev/null
  $EXEC timeout 45 ros2 topic pub -r 10 /arm/servo_temperatures braco_interfaces/msg/ServoTemperatures \
    "{celsius: [30, 30, 30, $HOT_C, 30, 30], valid: true}" >/dev/null 2>&1 &
  FAKE_PID=$!
  rest=""
  for _ in $(seq 1 40); do  # park: take control, move (up to 20 s), release, rest
    sleep 1
    rest=$(read_only)
    [[ $rest == true ]] && break
  done
  if [[ $rest == true ]]; then record PASS "HOT: park finished, rest mode on"; else record FAIL "HOT: rest mode did not come on"; fi
  if ask "Did the arm fold to the park pose and are the joints now free (can be moved by hand)?"; then
    record PASS "HOT: arm parked and joints released"
  else
    record FAIL "HOT: arm parked and joints released"
  fi
  kill "$FAKE_PID" 2>/dev/null
  unset FAKE_PID
  sleep 5
else
  record SKIP "HOT: park and rest (operator did not confirm the area)"
fi

say "6. make resume leaves rest mode, motion accepted again"
ros service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}" >/dev/null
sleep 1
if [[ $(read_only) == false ]]; then record PASS "resume: rest mode off"; else record FAIL "resume: rest mode off"; fi
sleep 5
if [[ $(read_only) == false ]]; then
  record PASS "resume on a cooled arm is not overruled (once per episode)"
else
  record FAIL "rest mode came back after resume"
fi

printf '\nbench: %d passed, %d failed (report: %s)\n' "$pass" "$fail" "$REPORT"
[[ $fail -eq 0 ]]
