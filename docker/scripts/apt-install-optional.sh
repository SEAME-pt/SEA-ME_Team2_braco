#!/usr/bin/env bash
# apt-install-optional <pkg>...
#
# Installs each package that exists in the configured apt repositories, one at a time, and
# reports (loudly, but without failing the build) the ones that do not. Intended for packages
# whose release status on a brand-new ROS distro may lag (e.g. gz_ros2_control, PlotJuggler).
# Run `apt-get update` first. Missing packages are recorded in /etc/seame/missing-packages.txt.
set -uo pipefail

missing=()

for pkg in "$@"; do
    if ! apt-cache show "${pkg}" >/dev/null 2>&1; then
        missing+=("${pkg}  (not in the apt repositories)")
        continue
    fi
    if ! apt-get install -y --no-install-recommends "${pkg}"; then
        missing+=("${pkg}  (install failed)")
    fi
done

if [ "${#missing[@]}" -gt 0 ]; then
    mkdir -p /etc/seame
    {
        echo "################################################################"
        echo "# WARNING: optional packages that could NOT be installed:"
        printf '#   - %s\n' "${missing[@]}"
        echo "# These are optional, so the build continues. See README, 'If the build fails'."
        echo "################################################################"
    } | tee -a /etc/seame/missing-packages.txt >&2
fi

exit 0
