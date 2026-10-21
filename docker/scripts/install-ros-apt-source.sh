#!/usr/bin/env bash
# install-ros-apt-source [VERSION|latest]
#
# Installs the ros2-apt-source package, i.e. step "Install the ros-apt-source packages" of
# "Installing on Ubuntu - how-to" in the ROS 2 Lyrical documentation.
#
# The documentation resolves the version through the GitHub REST API. That API allows only 60
# anonymous requests per hour per IP address (HTTP 403 afterwards), which a shared campus or office
# network exhausts quickly - and the failure then shows up as an empty version string and a 404.
# So by default this script uses a pinned, known-good version (no lookup at all). With `latest` it
# asks the API first and falls back to following the github.com "releases/latest" redirect, which
# is not subject to that limit.
#
# The package keeps itself up to date afterwards: it is part of the ROS apt repository, so a later
# `apt-get upgrade` replaces it when a new release is published.
set -euo pipefail

REQUESTED="${1:-${ROS_APT_SOURCE_VERSION:-1.3.0}}"
CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"

latest_version() {
    local v=""
    v="$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null \
            | grep -F tag_name | awk -F'"' '{print $4}')" || true
    if [ -z "${v}" ]; then
        echo "GitHub API unavailable or rate limited - following the releases/latest redirect instead" >&2
        v="$(curl -fsSLI --retry 3 -o /dev/null -w '%{url_effective}' \
                https://github.com/ros-infrastructure/ros-apt-source/releases/latest | sed 's|.*/||')" || true
    fi
    printf '%s' "${v}"
}

if [ "${REQUESTED}" = "latest" ]; then
    VERSION="$(latest_version)"
else
    VERSION="${REQUESTED}"
fi

if [ -z "${VERSION}" ]; then
    echo "ERROR: could not determine the ros-apt-source version (set ROS_APT_SOURCE_VERSION, e.g. 1.3.0)." >&2
    exit 1
fi

URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION}/ros2-apt-source_${VERSION}.${CODENAME}_all.deb"
echo "Installing ros2-apt-source ${VERSION} for ${CODENAME}"
if ! curl -fsSL --retry 5 --retry-delay 2 -o /tmp/ros2-apt-source.deb "${URL}"; then
    echo "ERROR: cannot download ${URL}" >&2
    echo "       Check network access to github.com, and that ${VERSION} has a '${CODENAME}' build." >&2
    exit 1
fi

dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb
