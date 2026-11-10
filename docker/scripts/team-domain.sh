#!/usr/bin/env bash
# team-domain.sh - the PC's ROS domain from the GitHub login, using docker/config/team-domains.tsv.
#
#   team-domain.sh set ENV_FILE   write PC_ROS_DOMAIN_ID for this user (make setup, PC only)
#   team-domain.sh check          check the table: 20-29, not 22/27/29, each login and domain once (make lint)
#
# The login comes from `gh api user`; TEAM_LOGIN=<login> skips gh. Not found (no gh, not logged in, not in the
# table): PC_ROS_DOMAIN_ID is left as it is (27 by default) with a warning, and setup goes on.
set -euo pipefail

TABLE="${TEAM_DOMAINS_TABLE:-$(cd "$(dirname "$0")/.." && pwd)/config/team-domains.tsv}"   # override: tests
ARM_DOMAIN=22       # ROS_DOMAIN_ID: the real arm
DEFAULT_DOMAIN=27   # PC_ROS_DOMAIN_ID when the login is not listed
TEST_DOMAIN=29      # braco_moveit_config's launch test
MIN=20
MAX=29

rows() { grep -vE '^[[:space:]]*(#|$)' "$TABLE" || true; }   # no rows: empty, not an error (check reports it)

check() {
    local fail=0 login name domain
    declare -A seen_login=() seen_domain=()
    [ -r "$TABLE" ] || { echo "team-domain: $TABLE not found" >&2; return 1; }
    [ -n "$(rows)" ] || { echo "team-domains.tsv: no teammates listed" >&2; return 1; }
    while IFS=$'\t' read -r login name domain; do
        if [ -z "$login" ] || [ -z "$name" ] || [ -z "$domain" ]; then
            echo "team-domains.tsv: '$login' needs 3 tab-separated columns: login name domain" >&2; fail=1; continue
        fi
        if ! [[ "$domain" =~ ^[0-9]+$ ]] || [ "$domain" -lt "$MIN" ] || [ "$domain" -gt "$MAX" ]; then
            echo "team-domains.tsv: $login: domain '$domain' is not between $MIN and $MAX" >&2; fail=1
        elif [ "$domain" -eq "$ARM_DOMAIN" ]; then
            echo "team-domains.tsv: $login: $ARM_DOMAIN is the real arm's domain (ROS_DOMAIN_ID)" >&2; fail=1
        elif [ "$domain" -eq "$DEFAULT_DOMAIN" ]; then
            echo "team-domains.tsv: $login: $DEFAULT_DOMAIN is the default for unlisted users" >&2; fail=1
        elif [ "$domain" -eq "$TEST_DOMAIN" ]; then
            echo "team-domains.tsv: $login: $TEST_DOMAIN is the MoveIt launch test's domain" >&2; fail=1
        fi
        local key="${login,,}"
        [ -z "${seen_login[$key]:-}" ] || { echo "team-domains.tsv: login $login is listed twice" >&2; fail=1; }
        [ -z "${seen_domain[$domain]:-}" ] || {
            echo "team-domains.tsv: domain $domain is given to ${seen_domain[$domain]} and $login" >&2; fail=1; }
        seen_login[$key]=1
        seen_domain[$domain]=$login
    done < <(rows)
    return "$fail"
}

set_env() {
    local env_file="$1" login="${TEAM_LOGIN:-}" current row name domain
    [ -f "$env_file" ] || { echo "team-domain: $env_file not found" >&2; return 1; }
    current=$(sed -n 's/^PC_ROS_DOMAIN_ID=//p' "$env_file" | tail -n 1)
    current=${current:-$DEFAULT_DOMAIN}
    check || { echo "team-domain: fix $TABLE first" >&2; return 1; }

    if [ -z "$login" ]; then
        if ! command -v gh >/dev/null 2>&1; then
            echo "warning: gh not installed: PC_ROS_DOMAIN_ID stays $current (install gh, gh auth login, make setup;" \
                 "or TEAM_LOGIN=<your GitHub login> make setup)" >&2; return 0
        fi
        if ! login=$(timeout 15 gh api user -q .login 2>/dev/null) || [ -z "$login" ]; then
            echo "warning: gh is not logged in: PC_ROS_DOMAIN_ID stays $current (gh auth login, then make setup)" >&2
            return 0
        fi
    fi

    row=$(rows | awk -F'\t' -v l="${login,,}" 'tolower($1) == l' | head -n 1)
    if [ -z "$row" ]; then
        echo "warning: GitHub login $login is not in docker/config/team-domains.tsv: PC_ROS_DOMAIN_ID stays $current" \
             "(add yourself there with a free domain, then make setup)" >&2; return 0
    fi
    IFS=$'\t' read -r _ name domain <<< "$row"

    if [ "$current" = "$domain" ]; then
        echo "PC_ROS_DOMAIN_ID=$domain ($name, GitHub $login): already set"; return 0
    fi
    if grep -q '^PC_ROS_DOMAIN_ID=' "$env_file"; then
        sed -i "s/^PC_ROS_DOMAIN_ID=.*/PC_ROS_DOMAIN_ID=$domain/" "$env_file"
    else
        echo "PC_ROS_DOMAIN_ID=$domain" >> "$env_file"
    fi
    echo "PC_ROS_DOMAIN_ID=$domain ($name, GitHub $login), was $current. Running containers: make restart (SIM=1)"
}

case "${1:-}" in
    check) check && echo "  team-domains.tsv: ok ($(rows | wc -l) teammates)" ;;
    set)   [ $# -eq 2 ] || { echo "usage: $0 set ENV_FILE" >&2; exit 2; }; set_env "$2" ;;
    *)     echo "usage: $0 set ENV_FILE | check" >&2; exit 2 ;;
esac
