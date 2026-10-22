#!/usr/bin/env bash
# Bench test of issue #55 (docs/ADR-0002-controle-unico.md, docs/ADR-0003-repouso-por-temperatura.md).
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
    /arm/read_only std_msgs/msg/Bool 2>/dev/null | sed -n 's/^data: *//p' | tr -d '\r'
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
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 0, source: 'script'}" >/dev/null 2>&1
  take script release >/dev/null 2>&1
  ros service call /arm/set_read_only std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
  [[ -n ${FAKE_PID:-} ]] && kill "$FAKE_PID" 2>/dev/null
}
trap cleanup EXIT

printf '\n# bench %s on %s\n' "$(date -Iseconds)" "$(hostname)" >>"$REPORT"
ros service list 2>/dev/null | grep -q /arm/take_control || { echo "arm_driver is not running: make up" >&2; exit 1; }

say "1. Control: free, taken, refused, forced"
take script release >/dev/null
expect_owner "" "the control starts free"
take web >/dev/null
expect_owner web "the page takes the control"
out=$(take joystick)
if grep -q "success=False" <<<"$out" && [[ $(owner) == web ]]; then
  record PASS "the joystick is refused while the page owns the control (stick ignored)"
else
  record FAIL "the joystick was not refused: $out"
fi
take joystick force >/dev/null
expect_owner joystick "force takes the control from the page"

say "2. Idle release after ${IDLE_S} s without motion"
take web force >/dev/null
sleep $((IDLE_S + 2))
expect_owner "" "arm_driver frees an idle owner"

say "3. STOP from anyone (the joystick owns the control)"
take joystick force >/dev/null
if ask "Is the arm powered and the area clear? A slow move will be started by the joystick owner"; then
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand \
    "{type: 1, targets: [{joint: 1, angle_deg: 10.0, speed: 10}], source: 'joystick'}" >/dev/null
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 0, source: 'web'}" >/dev/null
  if ask "Did the arm stop (the STOP came from 'web', not the owner)?"; then
    record PASS "STOP from a non-owner is obeyed"
  else
    record FAIL "STOP from a non-owner did not stop the arm"
  fi
else
  record SKIP "STOP from a non-owner (operator did not confirm the area)"
fi
take joystick release >/dev/null

say "4. Hand tests"
if ask "Page moving a joint: does the stick do nothing meanwhile?"; then
  record PASS "page moving, stick ignored"
else
  record FAIL "page moving, stick ignored"
fi
if ask "Hold button 8 for ~1 s: does the joystick take over (page shows it, answers 409)?"; then
  record PASS "hold button: joystick takes over"
else
  record FAIL "hold button: joystick takes over"
fi
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
    [[ $rest == True ]] && break
  done
  if [[ $rest == True ]]; then record PASS "HOT: park finished, rest mode on"; else record FAIL "HOT: rest mode did not come on"; fi
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
if [[ $(read_only) == False ]]; then record PASS "resume: rest mode off"; else record FAIL "resume: rest mode off"; fi
sleep 5
if [[ $(read_only) == False ]]; then
  record PASS "resume on a cooled arm is not overruled (once per episode)"
else
  record FAIL "rest mode came back after resume"
fi

printf '\nbench: %d passed, %d failed (report: %s)\n' "$pass" "$fail" "$REPORT"
[[ $fail -eq 0 ]]
