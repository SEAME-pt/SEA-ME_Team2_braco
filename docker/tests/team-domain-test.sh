#!/usr/bin/env bash
# Tests docker/scripts/team-domain.sh (make lint): writes PC_ROS_DOMAIN_ID from the login, keeps it when the
# login is unknown or gh is missing, refuses a broken table. No network, no gh: TEAM_LOGIN and a PATH without gh.
set -uo pipefail

SCRIPT="$(cd "$(dirname "$0")/../scripts" && pwd)/team-domain.sh"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
fails=0

domain() { sed -n 's/^PC_ROS_DOMAIN_ID=//p' "$TMP/env"; }
expect() {   # expect NAME WANTED_EXIT WANTED_DOMAIN EXIT
    if [ "$4" -eq "$2" ] && [ "$(domain)" = "$3" ]; then echo "  [ ok ] $1"
    else echo "  [FAIL] $1: exit $4 (want $2), PC_ROS_DOMAIN_ID=$(domain) (want $3)"; fails=1; fi
}
run() { bash "$SCRIPT" set "$TMP/env" >/dev/null 2>&1; }

printf 'login\tname\tdomain\nmagusk89\tMagusk\t24\nDenionline\tDaniel\t20\n' | tail -n +2 > "$TMP/table.tsv"
export TEAM_DOMAINS_TABLE="$TMP/table.tsv"
mkdir "$TMP/bin"   # every tool the script uses, but not gh
for t in grep sed awk head tail dirname timeout wc cat tr; do ln -s "$(type -P "$t")" "$TMP/bin/$t"; done

echo 'PC_ROS_DOMAIN_ID=27' > "$TMP/env"
TEAM_LOGIN=magusk89 run;   expect "listed login: set"                    0 24 $?
TEAM_LOGIN=magusk89 run;   expect "run again: unchanged"                 0 24 $?
TEAM_LOGIN=DENIONLINE run; expect "login case does not matter"           0 20 $?
TEAM_LOGIN=stranger run;   expect "unknown login: kept"                  0 20 $?
env -i PATH="$TMP/bin" HOME="$TMP" TEAM_DOMAINS_TABLE="$TEAM_DOMAINS_TABLE" "$BASH" "$SCRIPT" set "$TMP/env" \
    >/dev/null 2>&1;       expect "no gh: kept"                          0 20 $?
echo 'OTHER=1' > "$TMP/env"
TEAM_LOGIN=magusk89 run;   expect "no PC_ROS_DOMAIN_ID line: added"      0 24 $?
printf 'x\tX\t22\n' >> "$TMP/table.tsv"
TEAM_LOGIN=magusk89 run;   expect "table with 22: refused, kept"         1 24 $?
printf '# comments only\n' > "$TMP/table.tsv"
TEAM_LOGIN=magusk89 run;   expect "empty table: refused, kept"           1 24 $?

[ "$fails" -eq 0 ] && echo "  team-domain.sh: all tests pass"
exit "$fails"
