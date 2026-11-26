#!/usr/bin/env bash
# Fault rehearsal of issue #142: the arm stops (or holds safely) when a node dies, the network or the serial link
# drops, or a client sends bad data, and STOP still works during each fault.
#
# Run it with `make fault` on the Jetson with the system up, the arm powered and at a safe pose (make zero). The
# faults are injected by the script; the ones that need hands (the network cable, the page) ask the operator. Each
# motion is joint 1 at speed 10. The time to stop is measured on /joint_states by scripts/timing/fault_probe.py, run
# in the system container: from the fault until the pose stops changing (resolution: one /joint_states period).
#
# The arm does not move on its own: every section asks before it moves the arm. The two last sections kill
# ros2_control_node and arm_driver: the system must be restarted after them (make restart / make up).
#
# The result is printed, appended to fault-report.log, and written as a Markdown table (FAULT_DIR/results.md) for
# docs/guides/safety.md section 10.
#
# FAULT_EXEC         command prefix that runs a command inside the system container (set by `make fault`)
# FAULT_WEB_RESTART  command that starts the web page again after section 2 (set by `make fault`)
# FAULT_WEB_PORT     the page's port on this machine (set by `make fault`)

set -u

EXEC=${FAULT_EXEC:?run it with: make fault}
WEB_RESTART=${FAULT_WEB_RESTART:-}
WEB_PORT=${FAULT_WEB_PORT:-8000}
REPORT=${FAULT_REPORT:-fault-report.log}
DIR=${FAULT_DIR:-ros2_ws/bags/fault}      # on the host; /ws/bags/fault in the container
DIR_IN=${FAULT_DIR_IN:-/ws/bags/fault}
PROBE=${FAULT_PROBE:-/opt/braco/scripts/timing/fault_probe.py}
WATCHDOG_MAX_MS=${FAULT_WATCHDOG_MAX_MS:-400}   # command_timeout_ms (300) + a serial pass + a /joint_states period
STOP_MAX_MS=${FAULT_STOP_MAX_MS:-300}           # a STOP or deactivate until the pose stops changing
STOP_HOLD_S=${FAULT_STOP_HOLD_S:-3.5}           # arm_driver stop_hold_s (3 s) plus a margin
SWING=${FAULT_SWING_DEG:-25}                    # how far joint 1 moves in each test

pass=0
fail=0
RESULTS=$DIR/results.md

# Never from the terminal: docker exec would read the operator's answers meant for ask()
in_sys() { $EXEC "$@" </dev/null; }
ros() { in_sys ros2 "$@"; }
probe() { in_sys python3 "$PROBE" "$@"; }
say() { printf '\n== %s\n' "$*"; }
now() { date +%s.%N; }

record() {  # record PASS|FAIL|SKIP "what"
  printf '%-4s %s\n' "$1" "$2" | tee -a "$REPORT"
  case $1 in PASS) pass=$((pass + 1)) ;; FAIL) fail=$((fail + 1)) ;; esac
}

row() {  # row "fault" "result" "time to stop": one line of the results table
  printf '| %s | %s | %s |\n' "$1" "$2" "$3" >>"$RESULTS"
}

ask() {  # ask "question": y = yes
  local a
  read -r -p "$1 [y/N] " a
  [[ $a == y || $a == Y ]]
}

latched() {  # read /arm/stopped once: true / false
  ros topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /arm/stopped std_msgs/msg/Bool 2>/dev/null | sed -n 's/^data: *//p' | tr -d '\r'
}

owner() {
  ros topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /arm/control_owner std_msgs/msg/String 2>/dev/null | sed -n 's/^data: *//p' | tr -d "'\r"
}

lifecycle_state() {  # unconfigured, inactive, active... ('active [3]' from the CLI)
  ros lifecycle get /arm_driver 2>/dev/null | tr -d '\r' | awk '{print $1}'
}

kill_now() {  # kill_now PATTERN: SIGKILL in the container; prints the wall time of the kill
  # Timed in the container: docker exec takes ~0.5 s to start, which is no part of the fault
  # shellcheck disable=SC2016  # $1 expands in the container's bash, not here
  in_sys bash -c 'date +%s.%N; pkill -9 -f "$1"' _ "$1" | tr -d '\r' | head -1
}

kill_probes() {  # killing docker exec leaves the process running in the container
  in_sys pkill -9 -f "[f]ault_probe.py $1" 2>/dev/null
}

wait_mark() {  # wait_mark FILE MARK [TIMEOUT_S]: until the probe wrote "# MARK"
  local i
  for ((i = 0; i < ${3:-20} * 10; i++)); do
    grep -q "^# $2 " "$1" 2>/dev/null && return 0
    sleep 0.1
  done
  return 1
}

watch_start() {  # watch_start NAME SECONDS [probe watch options]: record /joint_states into NAME.txt
  local name=$1 secs=$2
  shift 2
  probe watch --duration "$secs" "$@" >"$DIR/$name.txt" 2>/dev/null &
  WATCH_PID=$!
  wait_mark "$DIR/$name.txt" ready || { echo "fault: no /joint_states for the record" >&2; exit 1; }
}

stream_start() {  # stream_start NAME [probe stream options]: a client resending one target
  local name=$1
  shift
  probe stream "$@" >"$DIR/$name.stream" 2>&1 &
  wait_mark "$DIR/$name.stream" streaming || { cat "$DIR/$name.stream" >&2; exit 1; }
}

analyse() {  # analyse NAME [SINCE]: "moving=.. stop_ms=.. latched_ms=.. travel_deg=.."
  probe analyse "$DIR_IN/$1.txt" ${2:+--since "$2"} 2>/dev/null | tr -d '\r'
}
field() { sed -n "s/.*$2=\([^ ]*\).*/\1/p" <<<"$1"; }   # field "LINE" NAME

j1_deg() {  # joint 1 now, in degrees, rounded
  probe pose 2>/dev/null | tr -d '\r' | awk '{printf "%.0f", $1}'
}

target_away() {  # a joint 1 target SWING deg away from now, toward 0 (keeps away from the limits)
  local j1
  j1=$(j1_deg)
  if ((j1 > 0)); then echo $((j1 - SWING)); else echo $((j1 + SWING)); fi
}

resume() {  # release the STOP after the hold; the arm does not move
  local out
  sleep "$STOP_HOLD_S"
  out=$(ros service call /arm/release_stop std_srvs/srv/Trigger 2>&1 | tr -d '\r')
  if grep -q "success=True" <<<"$out"; then return 0; fi
  record FAIL "release_stop after the test: $out"
  exit 1
}

stop_during() {  # stop_during "fault": STOP from a third client while the fault goes on, then resume
  local r ms lat
  watch_start "stop-$STOP_N" 6 --action stop --after 1.0
  wait "$WATCH_PID"
  r=$(analyse "stop-$STOP_N")
  STOP_N=$((STOP_N + 1))
  ms=$(field "$r" stop_ms)
  lat=$(field "$r" latched_ms)
  if [[ $lat != none && $ms != none ]] && ((${ms%.*} <= STOP_MAX_MS)); then
    record PASS "STOP during: $1 (still after $ms ms, latched after $lat ms)"
    row "STOP during: $1" "latched, arm still" "$ms ms"
  else
    record FAIL "STOP during: $1 ($r)"
    row "STOP during: $1" "**FAIL**: $r" "-"
  fi
  kill_probes stream
  resume
}
STOP_N=1

reactivate_mission_path() {  # after a STOP, braco_hardware and arm_controller are inactive (braco_mission does this)
  ros control set_hardware_component_state BracoSystem active >/dev/null 2>&1
  ros control switch_controllers --activate arm_controller >/dev/null 2>&1
}

cleanup() {
  kill_probes stream
  kill_probes trajectory
  kill_probes watch
  [[ $(lifecycle_state) == inactive ]] && ros lifecycle set /arm_driver activate >/dev/null 2>&1
  # HALT, not STOP: the rehearsal ending is no emergency and must not leave the arm latched
  ros topic pub --once /arm/command braco_interfaces/msg/ArmCommand "{type: 3, source: 'script'}" >/dev/null 2>&1
  [[ $(latched) == true ]] && echo "fault: STOP is still latched: make resume" >&2
  printf '\nfault: %d passed, %d failed (report: %s, table: %s)\n' "$pass" "$fail" "$REPORT" "$RESULTS"
}

mkdir -p "$DIR"
printf '\n# fault rehearsal %s on %s\n' "$(date -Iseconds)" "$(hostname)" >>"$REPORT"
printf '# Fault rehearsal %s\n\n| Fault | Result | Time to stop |\n|---|---|---|\n' "$(date -I)" >"$RESULTS"
ros service list 2>/dev/null | grep -q /arm/take_control || { echo "arm_driver is not running: make up" >&2; exit 1; }
[[ $(lifecycle_state) == active ]] || { echo "arm_driver is not active: ros2 lifecycle set /arm_driver activate" >&2; exit 1; }
[[ $(latched) == true ]] && { echo "STOP is latched: make resume first" >&2; exit 1; }
trap cleanup EXIT
echo "Each section moves joint 1 by ${SWING} deg at speed 10. Keep a hand near the power switch."

say "1. A client dies mid-move: the watchdog stops the arm"
if ask "Joint 1 will move. Is the area clear?"; then
  watch_start client 12
  stream_start client --source web --take --angle "$(target_away)"
  sleep 1.5
  t0=$(kill_now '[f]ault_probe.py stream')
  wait "$WATCH_PID"
  r=$(analyse client "$t0")
  ms=$(field "$r" stop_ms)
  if [[ $(field "$r" moving) == yes && $ms != none ]] && ((${ms%.*} <= WATCHDOG_MAX_MS)); then
    record PASS "client killed mid-move: still after $ms ms"
    row "Client (\`web\` stream) killed mid-move" "watchdog stopped the arm" "$ms ms"
  else
    record FAIL "client killed mid-move: $r"
    row "Client (\`web\` stream) killed mid-move" "**FAIL**: $r" "-"
  fi
  # A dead client again, then a STOP before the watchdog: the stream goes on in a second client
  stream_start client2 --source web --take --angle "$(target_away)"
  stop_during "a client streaming"
else
  record SKIP "client killed mid-move (operator did not confirm the area)"
fi

say "2. The web page dies mid-move"
if [[ -z $WEB_RESTART ]] || ! curl -fsS -m 2 -o /dev/null "http://localhost:$WEB_PORT/api/status" 2>/dev/null; then
  record SKIP "page killed mid-move: the page is not running (make web)"
elif ask "Start a slow move of one joint on the page now (http://<jetson-ip>:8000). Is it moving?"; then
  watch_start page 8
  t0=$(kill_now '[u]vicorn web.server.app')
  wait "$WATCH_PID"
  r=$(analyse page "$t0")
  ms=$(field "$r" stop_ms)
  if [[ $ms != none ]] && ((${ms%.*} <= WATCHDOG_MAX_MS)); then
    record PASS "page killed mid-move: still after $ms ms ($r)"
    row "Web page server killed mid-move" "watchdog stopped the arm" "$ms ms"
  else
    record FAIL "page killed mid-move: $r"
    row "Web page server killed mid-move" "**FAIL**: $r" "-"
  fi
  $WEB_RESTART </dev/null >/dev/null 2>&1 && echo "the page is running again"
else
  record SKIP "page killed mid-move (no move on the page)"
fi

say "3. Bad data: the arm must not move; STOP during each"
bad() {  # bad "fault" NAME [stream options]: a bad stream for 6 s, then STOP during it
  local what=$1 name=$2 r travel
  shift 2
  watch_start "$name" 7
  stream_start "$name" "$@"
  wait "$WATCH_PID"
  r=$(analyse "$name")
  travel=$(field "$r" travel_deg)
  if [[ -n $travel ]] && awk -v t="$travel" 'BEGIN {exit !(t < 1.0)}'; then
    record PASS "$what: the arm stayed still (moved $travel deg)"
    row "$what" "ignored, arm still" "- (no motion)"
  else
    record FAIL "$what: the arm moved ($r)"
    row "$what" "**FAIL**: the arm moved $travel deg" "-"
  fi
  stop_during "$what"
}
if ask "Bad commands will be sent; joint 1 must NOT move. A bug would move it: is the area clear?"; then
  j1=$(j1_deg)
  bad "NaN angles from the owner" nan --source web --take --angle 0 --nan
  # The owner holds the pose; an intruder streams a real target
  stream_start hold --source web --take --angle "$j1"
  bad "Motion from a wrong \`source\`" intruder --source intruder --angle "$(target_away)"
  kill_probes stream
  read -r -a pose <<<"$(probe pose 2>/dev/null | tr -d '\r')"
  if ((${#pose[@]} == 6)) && ask "Last: a pose with joint 2 at -130 deg (into the table) is sent. arm_driver must refuse it. Hand on the power switch?"; then
    bad "Pose below the table (J2 -130 deg)" table --source web --take \
      --all "${pose[0]}" -130 "${pose[2]}" "${pose[3]}" "${pose[4]}" "${pose[5]}"
  else
    record SKIP "pose below the table"
  fi
else
  record SKIP "bad data (operator did not confirm the area)"
fi
record SKIP "angle out of the limits: clamped to the joint limit, so it moves the arm there (gtest AnglesOutOfTheLimitsAreClamped)"
row "Angle out of the limits" "clamped to the joint limit (gtest only: it moves the arm to the limit)" "-"

say "4. arm_driver deactivated mid-move (#95)"
if ask "Joint 1 will move, then arm_driver is deactivated. Is the area clear?"; then
  stream_start deact --source web --take --angle "$(target_away)"
  watch_start deactivate 6 --action deactivate --after 1.0
  wait "$WATCH_PID"
  r=$(analyse deactivate)
  ms=$(field "$r" stop_ms)
  if [[ $(field "$r" latched_ms) != none && $ms != none ]] && ((${ms%.*} <= STOP_MAX_MS)); then
    record PASS "deactivate mid-move: still after $ms ms, latched"
    row "\`ros2 lifecycle set /arm_driver deactivate\` mid-move" "stopped and latched" "$ms ms"
  else
    record FAIL "deactivate mid-move: $r"
    row "\`ros2 lifecycle set /arm_driver deactivate\` mid-move" "**FAIL**: $r" "-"
  fi
  kill_probes stream
  ros lifecycle set /arm_driver activate >/dev/null 2>&1
  resume
else
  record SKIP "deactivate mid-move (operator did not confirm the area)"
fi

say "5. The mission client (move_group) dies mid-trajectory"
trajectory_start() {  # trajectory_start NAME DELTA: one arm_controller goal of 8 s, as `mission`
  probe trajectory --joint 1 --delta "$2" --seconds 8 >"$DIR/$1.traj" 2>&1 &
  wait_mark "$DIR/$1.traj" goal_accepted 30 || { cat "$DIR/$1.traj" >&2; record FAIL "$1: goal refused"; return 1; }
}
delta() {  # joint 1 swing toward 0, for a trajectory
  if (($(j1_deg) > 0)); then echo "-$SWING"; else echo "$SWING"; fi
}
if ask "Joint 1 will follow an 8 s trajectory through arm_controller. Is the area clear?"; then
  reactivate_mission_path
  d=$(delta)
  watch_start mission 16
  if trajectory_start mission "$d"; then
    sleep 3
    t0=$(kill_now '[f]ault_probe.py trajectory')
    wait "$WATCH_PID"
    r=$(analyse mission "$t0")
    ms=$(field "$r" stop_ms)
    # The goal lives in arm_controller: it runs to its end (5 s left) and stops there
    if [[ $ms != none ]] && ((${ms%.*} <= 6500)); then
      record PASS "mission client killed: the goal ran to its end, still after $ms ms"
      row "Mission client (\`move_group\`) killed mid-trajectory" "the goal runs to its end in \`arm_controller\`, then the arm holds" "$ms ms (end of the goal)"
    else
      record FAIL "mission client killed: $r"
      row "Mission client (\`move_group\`) killed mid-trajectory" "**FAIL**: $r" "-"
    fi
  fi
  reactivate_mission_path
  if trajectory_start mission-stop "$(delta)"; then
    sleep 1
    stop_during "a trajectory (arm_controller)"
    kill_probes trajectory
  fi
else
  record SKIP "mission client killed (operator did not confirm the area)"
fi

say "6. The network between the PC and the Jetson is cut mid-move"
if ask "On the PC, start a slow move (MoveIt goal with make moveit, or the page in the PC's browser). Is it moving?"; then
  watch_start network 30
  read -r -p "Pull the PC's network cable (or turn its Wi-Fi off) and press Enter at the same time... " _
  t0=$(now)
  sleep 1
  stop_during "the network cut (STOP sent from the Jetson)"
  wait "$WATCH_PID"
  r=$(analyse network "$t0")
  ms=$(field "$r" stop_ms)
  if [[ $ms != none ]] && ask "Did the arm stop, and stay still?"; then
    record PASS "network cut: still after $ms ms"
    row "Network PC-Jetson cut mid-move" "the Jetson side stops the move; STOP from the Jetson works" "$ms ms"
  else
    record FAIL "network cut: $r"
    row "Network PC-Jetson cut mid-move" "**FAIL**: $r" "-"
  fi
  read -r -p "Plug the PC back in, check the page shows the STOP released, and press Enter... " _
else
  record SKIP "network cut (no move from the PC)"
fi

say "7. ros2_control_node dies mid-trajectory (the system must be restarted after it)"
if ask "Joint 1 will follow a trajectory, then ros2_control_node is killed. Is the area clear?"; then
  reactivate_mission_path
  d=$(delta)
  watch_start controller 16
  if trajectory_start controller "$d"; then
    sleep 3
    t0=$(kill_now '[r]os2_control_node')
    wait "$WATCH_PID"
    r=$(analyse controller "$t0")
    ms=$(field "$r" stop_ms)
    if [[ $(field "$r" moving) == yes && $ms != none ]] && ((${ms%.*} <= WATCHDOG_MAX_MS)); then
      record PASS "controller killed: still after $ms ms"
      row "\`ros2_control_node\` killed mid-trajectory" "\`write()\` stops, the watchdog stops the arm" "$ms ms"
    else
      record FAIL "controller killed: $r"
      row "\`ros2_control_node\` killed mid-trajectory" "**FAIL**: $r" "-"
    fi
    kill_probes trajectory
    stream_start after-controller --source web --take --angle "$(target_away)"
    stop_during "a client streaming after the controller died"
  fi
else
  record SKIP "controller killed (operator did not confirm the area)"
fi

record SKIP "serial cable unplugged: wired inside the base; latch on feedback loss covered by gtests (ADR-0007)"
row "Serial link lost (cable inside the base)" "STOP latched after 1 s without feedback, release refused until it is back (gtest, ADR-0007)" "the move in progress ends on the Atom"

say "8. arm_driver dies mid-move (last: the system stops with it)"
if ask "Joint 1 will move, then arm_driver is killed (SIGKILL). Watch the arm. Is the area clear?"; then
  stream_start driver --source web --take --angle "$(target_away)"
  sleep 1.5
  kill_now '[l]ib/braco_driver/arm_driver' >/dev/null
  echo "arm_driver killed. No /joint_states from now on: watch the arm."
  if ask "Did the arm stop at its last target and hold it (motors on), and nothing moved after?"; then
    record PASS "arm_driver killed: the arm finished its last target and held"
    row "\`arm_driver\` killed (SIGKILL) mid-move" "the arm finishes its last target and holds; the launch shuts down" "the last target (no feedback)"
  else
    record FAIL "arm_driver killed: the arm did not hold (write down what it did)"
    row "\`arm_driver\` killed (SIGKILL) mid-move" "**FAIL**: see fault-report.log" "-"
  fi
  trap - EXIT
  printf '\nfault: %d passed, %d failed (report: %s, table: %s)\n' "$pass" "$fail" "$REPORT" "$RESULTS"
  echo "The system is down with arm_driver: make up"
  [[ $fail -eq 0 ]]
  exit
else
  record SKIP "arm_driver killed (operator did not confirm the area)"
fi

[[ $fail -eq 0 ]]
