#!/usr/bin/env bash
# ROS 2 talker/listener test - the "is ROS 2 really working here?" check, for the PC and the Jetson.
#
#   talker-listener.sh selftest   C++ talker + Python listener inside this container -> PASS/FAIL
#   talker-listener.sh talker     run the C++ talker until Ctrl-C        (start it on machine A ...)
#   talker-listener.sh listener   run the Python listener and check it   (... and this on machine B)
#
# Why these two nodes: they are the ones the official ROS 2 install page uses. One is C++ (rclcpp) and the other
# Python (rclpy), so a pass shows both client libraries, the type support and the selected RMW working together.
#
# The self-test proves ROS 2 works *inside one container*. The two-machine run (talker here, listener there) is
# what proves DDS discovery across the network: same ROS_DOMAIN_ID, same RMW, multicast/firewall in order.
#
# Environment (all optional; the Makefile passes them through):
#   TOPIC    selftest: a private unique topic, so a talker elsewhere on the network cannot make the test pass by
#            accident. talker/listener: /chatter, the default of the ROS demos.
#   COUNT    messages the listener must hear (default 5). For `listener`, COUNT=0 means "just run until Ctrl-C".
#   TIMEOUT  seconds to wait for them (default 30)
#
# Exit status: 0 PASS, 1 FAIL, 2 cannot run (ROS or the demo packages missing, bad arguments).
set -uo pipefail

# ---- process handling (first thing in the script: the traps must exist before any child process runs) ----
# `ros2 run` starts the node as a child and, on SIGINT, only waits for it. Signalling the `ros2 run` process
# alone would therefore leave the node running as an orphan that keeps publishing and pollutes the next test.
# Every node gets its own process group (setsid), and the whole group is signalled.
PIDS=()
TMP=""

# A background job started by a non-interactive shell inherits "SIGINT ignored", which would make Ctrl-C
# ineffective on the node until it installs its own handler. Restore the default disposition before exec.
start_node() {   # start_node LOGFILE COMMAND...   (sets LAST_PID)
    local log="$1"; shift
    if command -v python3 >/dev/null 2>&1; then
        setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' "$@" >"${log}" 2>&1 &
    else
        setsid "$@" >"${log}" 2>&1 &
    fi
    LAST_PID=$!
    PIDS+=("${LAST_PID}")
}
group_alive() { kill -0 -- "-$1" 2>/dev/null; }
stop_nodes() {     # SIGINT to every node group, up to 3 s of grace, then SIGKILL: always bounded
    local pid alive
    for pid in "${PIDS[@]:-}"; do [ -n "${pid}" ] && kill -INT -- "-${pid}" 2>/dev/null; done
    for _ in $(seq 1 30); do
        alive=0
        for pid in "${PIDS[@]:-}"; do [ -n "${pid}" ] && group_alive "${pid}" && alive=1; done
        [ "${alive}" -eq 0 ] && break
        sleep 0.1
    done
    for pid in "${PIDS[@]:-}"; do [ -n "${pid}" ] && kill -KILL -- "-${pid}" 2>/dev/null; done
    PIDS=()
}
cleanup() { stop_nodes; if [ -n "${TMP}" ]; then rm -rf "${TMP}"; fi; }
trap cleanup EXIT
trap 'exit 130' INT TERM
TMP="$(mktemp -d)"

MODE="${1:-selftest}"
COUNT="${COUNT:-5}"
TIMEOUT="${TIMEOUT:-30}"
ROLE_TAG="${SEAME_ROLE:-host}"
NODE_SUFFIX="${ROLE_TAG//[^A-Za-z0-9]/_}"          # node names may not contain '-'

ok()      { printf '  [ ok ] %s\n' "$*"; }
warn()    { printf '  [warn] %s\n' "$*"; }
bad()     { printf '  [FAIL] %s\n' "$*"; }
section() { printf '\n== %s ==\n' "$*"; }
strip()   { sed 's/\x1b\[[0-9;]*m//g'; }              # the image forces coloured ROS logs

case "${COUNT}" in ''|*[!0-9]*) bad "COUNT must be a whole number (got '${COUNT}')"; exit 2 ;; esac
case "${TIMEOUT}" in ''|*[!0-9]*) bad "TIMEOUT must be a whole number (got '${TIMEOUT}')"; exit 2 ;; esac

# The Makefile enters through the image entrypoint, which has already sourced ROS. Do it here as a fallback.
if ! command -v ros2 >/dev/null 2>&1 && [ -r /usr/local/share/seame/env.sh ]; then
    # shellcheck disable=SC1091
    . /usr/local/share/seame/env.sh
fi
if ! command -v ros2 >/dev/null 2>&1; then
    bad "ros2 not found - run this through the container (make talker-listener), which sources ROS first"
    exit 2
fi

# ---- helpers ----------------------------------------------------------------------------------------
elapsed() { awk -v a="$(date +%s.%N)" -v b="$1" 'BEGIN { printf "%.1f", a - b }'; }

preflight() {
    section "Environment"
    ok "role=${ROLE_TAG}  host=$(hostname)  ROS_DISTRO=${ROS_DISTRO:-unset}"
    ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<distro default>}  discovery=${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
    local missing=0 pkg
    for pkg in demo_nodes_cpp demo_nodes_py; do
        if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then ok "package ${pkg}"; else bad "package ${pkg} is not installed"; missing=1; fi
    done
    if [ "${missing}" -ne 0 ]; then
        echo "       install for this session: sudo apt-get update && sudo apt-get install -y ros-${ROS_DISTRO:-lyrical}-demo-nodes-cpp ros-${ROS_DISTRO:-lyrical}-demo-nodes-py"
        echo "       (the jetson-driver image includes them; rebuild with: make build)"
        exit 2
    fi
}

wait_for_messages() {   # wait_for_messages LOG WANT LIMIT LISTENER_PID  -> sets HEARD FIRST_AT ; returns 0 ok, 1 timeout, 2 listener died
    local log="$1" want="$2" limit="$3" lpid="$4" shown=0 total t0
    t0="$(date +%s.%N)"; HEARD=0; FIRST_AT=""
    while :; do
        HEARD="$(grep -c 'I heard' "${log}" 2>/dev/null || true)"; HEARD="${HEARD:-0}"
        total="$(wc -l < "${log}" 2>/dev/null || echo 0)"
        if [ "${total}" -gt "${shown}" ]; then
            sed -n "$((shown + 1)),${total}p" "${log}" | strip | sed 's/^/    | /'
            shown="${total}"
        fi
        if [ -z "${FIRST_AT}" ] && [ "${HEARD}" -gt 0 ]; then FIRST_AT="$(elapsed "${t0}")"; fi
        [ "${HEARD}" -ge "${want}" ] && return 0
        if ! group_alive "${lpid}"; then return 2; fi
        awk -v e="$(elapsed "${t0}")" -v l="${limit}" 'BEGIN { exit !(e >= l) }' && return 1
        sleep 0.2
    done
}

sequence_report() {     # sequence_report LOG -> "first last count gaps duplicates"
    strip < "$1" | sed -n 's/.*I heard: \[Hello World: \([0-9][0-9]*\)\].*/\1/p' | awk '
        NR == 1 { first = $1 }
        NR > 1  { d = $1 - prev; if (d == 0) dupes++; else if (d != 1) gaps++ }
        { prev = $1; last = $1; n = NR }
        END { printf "%d %d %d %d %d\n", first, last, n, gaps + 0, dupes + 0 }'
}

publishers_of() {       # publishers_of TOPIC -> one fully qualified node name per line
    timeout 15 ros2 topic info "$1" -v 2>/dev/null | awk '
        /^Node name:/      { name = $3 }
        /^Node namespace:/ { ns = $3 }
        /^Endpoint type:/  { if ($3 == "PUBLISHER") print (ns == "/" ? "" : ns) "/" name }'
}

hints_no_messages() {
    echo "       Nothing was heard. Check, in this order:"
    echo "         1. is a talker running?  On the other machine:  make talker   (same TOPIC=${TOPIC})"
    echo "         2. both machines use the same ROS_DOMAIN_ID (here: ${ROS_DOMAIN_ID:-0}) and RMW_IMPLEMENTATION (here: ${RMW_IMPLEMENTATION:-default})"
    echo "         3. network: same subnet, UDP not blocked by a firewall; if multicast is blocked set ROS_STATIC_PEERS in compose.yml"
    echo "         4. what this container can see:  make exec CMD='ros2 node list; ros2 topic list'"
}

# ---- modes ------------------------------------------------------------------------------------------
mode_talker() {
    TOPIC="${TOPIC:-/chatter}"
    local node="talker_${NODE_SUFFIX}"
    section "ROS 2 talker (C++) on ${ROLE_TAG}"
    preflight
    echo
    echo "  Publishing 'Hello World: N' once a second on ${TOPIC} as node /${node}."
    echo "  Now start the listener on the other machine (or this one):  make listener"
    echo "  Ctrl-C stops the talker."
    echo
    exec ros2 run demo_nodes_cpp talker --ros-args -r "chatter:=${TOPIC}" -r "__node:=${node}"
}

mode_listener() {
    TOPIC="${TOPIC:-/chatter}"
    local node="listener_${NODE_SUFFIX}"
    section "ROS 2 listener (Python) on ${ROLE_TAG}"
    preflight
    if [ "${COUNT}" -eq 0 ]; then
        echo; echo "  COUNT=0: listening on ${TOPIC} until Ctrl-C, no pass/fail check."; echo
        exec ros2 run demo_nodes_py listener --ros-args -r "chatter:=${TOPIC}" -r "__node:=${node}"
    fi

    section "Listening on ${TOPIC} (need ${COUNT} messages within ${TIMEOUT} s)"
    start_node "${TMP}/listener.log" ros2 run demo_nodes_py listener --ros-args -r "chatter:=${TOPIC}" -r "__node:=${node}"
    local lpid="${LAST_PID}" rc=0
    wait_for_messages "${TMP}/listener.log" "${COUNT}" "${TIMEOUT}" "${lpid}" || rc=$?

    section "Result"
    local pubs
    if [ "${HEARD}" -gt 0 ]; then pubs="$(publishers_of "${TOPIC}")"; else pubs=""; fi
    if [ "${rc}" -eq 0 ]; then
        ok "heard ${HEARD} messages on ${TOPIC} (needed ${COUNT}; first after ${FIRST_AT} s)"
        local p tag any_remote=0
        while IFS= read -r p; do
            [ -n "${p}" ] || continue
            case "${p}" in *"_${NODE_SUFFIX}") tag="this machine" ;; *) tag="ANOTHER machine/role"; any_remote=1 ;; esac
            ok "publisher ${p}  (${tag})"
        done <<< "${pubs}"
        [ -n "${pubs}" ] || warn "could not read the publisher list (messages did arrive)"
        read -r first last n gaps dupes < <(sequence_report "${TMP}/listener.log")
        if [ "${gaps}" -eq 0 ] && [ "${dupes}" -eq 0 ]; then ok "sequence intact: ${first}..${last} (${n} messages, no gaps or duplicates)"
        else warn "sequence ${first}..${last}: ${gaps} gap(s), ${dupes} duplicate(s) over ${n} messages"; fi
        echo
        if [ "${any_remote}" -eq 1 ]; then echo "PASS: received from another machine over ${RMW_IMPLEMENTATION:-the default RMW} - cross-machine ROS 2 works"
        else echo "PASS: received from a talker on this machine (start the talker on the other machine to test the network)"; fi
        exit 0
    fi
    if [ "${rc}" -eq 2 ]; then bad "the listener process exited early:"; strip < "${TMP}/listener.log" | tail -n 8 | sed 's/^/       /'
    else bad "heard ${HEARD}/${COUNT} messages in ${TIMEOUT} s"; hints_no_messages; fi
    echo; echo "FAIL"; exit 1
}

mode_selftest() {
    TOPIC="${TOPIC:-/seame_selftest_$$}"
    local tnode="selftest_talker_${NODE_SUFFIX}" lnode="selftest_listener_${NODE_SUFFIX}"
    section "ROS 2 talker/listener self-test on ${ROLE_TAG}"
    preflight

    section "Running (private topic ${TOPIC}; need ${COUNT} messages within ${TIMEOUT} s)"
    echo "  talker   /${tnode}   C++    demo_nodes_cpp"
    echo "  listener /${lnode}   Python demo_nodes_py"
    start_node "${TMP}/talker.log" ros2 run demo_nodes_cpp talker --ros-args -r "chatter:=${TOPIC}" -r "__node:=${tnode}"
    local tpid="${LAST_PID}"
    start_node "${TMP}/listener.log" ros2 run demo_nodes_py listener --ros-args -r "chatter:=${TOPIC}" -r "__node:=${lnode}"
    local lpid="${LAST_PID}" rc=0
    wait_for_messages "${TMP}/listener.log" "${COUNT}" "${TIMEOUT}" "${lpid}" || rc=$?

    # Look at the graph while both nodes are still alive.
    local nodes=""
    nodes="$(timeout 15 ros2 node list 2>/dev/null || true)"
    local talker_alive=1; group_alive "${tpid}" || talker_alive=0

    section "Results"
    local failed=0
    if [ "${rc}" -eq 0 ]; then ok "heard ${HEARD} messages (needed ${COUNT}; first after ${FIRST_AT} s)"
    elif [ "${rc}" -eq 2 ]; then bad "the listener process exited early:"; strip < "${TMP}/listener.log" | tail -n 6 | sed 's/^/       /'; failed=1
    else bad "heard only ${HEARD}/${COUNT} messages in ${TIMEOUT} s"; failed=1; fi

    if [ "${talker_alive}" -eq 0 ]; then bad "the talker process exited early:"; strip < "${TMP}/talker.log" | tail -n 6 | sed 's/^/       /'; failed=1
    else
        local published; published="$(grep -c 'Publishing' "${TMP}/talker.log" 2>/dev/null || true)"
        ok "talker is alive and has published ${published:-0} messages"
    fi

    if [ "${HEARD}" -gt 0 ]; then
        read -r first last n gaps dupes < <(sequence_report "${TMP}/listener.log")
        if [ "${gaps}" -eq 0 ] && [ "${dupes}" -eq 0 ]; then ok "sequence intact: ${first}..${last} (no gaps or duplicates)"
        else warn "sequence ${first}..${last}: ${gaps} gap(s), ${dupes} duplicate(s) - unexpected for reliable QoS on one host"; fi
    fi

    if grep -q "/${tnode}\$" <<< "${nodes}" && grep -q "/${lnode}\$" <<< "${nodes}"; then ok "both nodes are visible in the ROS graph"
    else warn "ros2 node list did not show both nodes (the CLI daemon can lag a few seconds; the messages above are what counts)"; fi

    echo
    if [ "${failed}" -eq 0 ]; then
        echo "PASS: C++ talker -> Python listener over ${RMW_IMPLEMENTATION:-the default RMW} works on ${ROLE_TAG}"
        echo "      Next: prove the network - 'make talker' on one machine, 'make listener' on the other."
        exit 0
    fi
    [ "${HEARD}" -eq 0 ] && echo "       No message arrived even on this machine: check the RMW and the container's network (make info, make doctor)."
    echo; echo "FAIL"; exit 1
}

case "${MODE}" in
    selftest) mode_selftest ;;
    talker)   mode_talker ;;
    listener) mode_listener ;;
    -h|--help|help) sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//' ;;
    *) bad "unknown mode '${MODE}' (use: selftest | talker | listener)"; exit 2 ;;
esac
