#!/usr/bin/env bash
# install-plotjuggler
#
# Installs PlotJuggler 4 from https://github.com/PlotJuggler/PlotJuggler (release .deb) plus the
# "ROS 2 Topic Subscriber" extension from https://github.com/PlotJuggler/pj-official-plugins.
#
# Why two downloads: the .deb bundles its own Qt6 and a ROS 2 subscriber, but the bundled one (1.0.2)
# only has humble/iron/jazzy/rolling builds. Extension 1.1.1 adds `lyrical`, and PlotJuggler picks the
# build matching $ROS_DISTRO at run time. Both files are pinned by SHA-256, so a changed or tampered
# download fails the check instead of being installed.
#
# Like the other optional pieces this never fails the image build: problems are written to
# /etc/seame/missing-packages.txt, which `seame-smoke-test` prints. Needs root and an existing `dev` user.
#
# To upgrade: set new versions and the new checksums (the extension publishes a .sha256 next to its zip;
# for the .deb run sha256sum on the download) via the PLOTJUGGLER_* / PJ_ROS2_EXTENSION_* build args.
set -uo pipefail

VERSION="${PLOTJUGGLER_VERSION:?PLOTJUGGLER_VERSION is required}"
DEB_SHA256="${PLOTJUGGLER_DEB_SHA256:?PLOTJUGGLER_DEB_SHA256 is required}"
EXT_VERSION="${PJ_ROS2_EXTENSION_VERSION:?PJ_ROS2_EXTENSION_VERSION is required}"
EXT_SHA256="${PJ_ROS2_EXTENSION_SHA256:?PJ_ROS2_EXTENSION_SHA256 is required}"
USER_HOME="/home/dev"
EXT_DIR="${USER_HOME}/.local/share/PlotJuggler/PlotJuggler4/extensions"
WORK="$(mktemp -d)"
trap 'rm -rf "${WORK}"' EXIT

note() {
    mkdir -p /etc/seame
    printf '%s\n' "PlotJuggler 4: $*" | tee -a /etc/seame/missing-packages.txt >&2
}

fetch() {   # fetch URL OUTPUT
    curl -fsSL --retry 5 --retry-delay 2 -o "$2" "$1"
}

verify() {  # verify FILE SHA256
    echo "$2  $1" | sha256sum -c --status
}

# --- 1. the application ---------------------------------------------------------------------
if [ "$(dpkg --print-architecture)" != "amd64" ]; then
    note "skipped: the release .deb is amd64-only (this image is $(dpkg --print-architecture)); use Foxglove Studio or rqt_plot"
    exit 0
fi

DEB_URL="https://github.com/PlotJuggler/PlotJuggler/releases/download/${VERSION}/plotjuggler4_${VERSION}-1_amd64.deb"
echo "Downloading ${DEB_URL}"
if ! fetch "${DEB_URL}" "${WORK}/plotjuggler4.deb"; then
    note "not installed: cannot download ${DEB_URL}"
    exit 0
fi
if ! verify "${WORK}/plotjuggler4.deb" "${DEB_SHA256}"; then
    note "not installed: checksum mismatch for the ${VERSION} .deb (expected ${DEB_SHA256})"
    exit 0
fi
# apt resolves the few system libraries the self-contained package still needs (e.g. libopengl0).
if ! { apt-get update -qq && apt-get install -y --no-install-recommends "${WORK}/plotjuggler4.deb"; }; then
    note "not installed: apt could not install the ${VERSION} .deb"
    rm -rf /var/lib/apt/lists/*
    exit 0
fi
rm -rf /var/lib/apt/lists/*

# --- 2. the ROS 2 subscriber extension with a lyrical build ------------------------------------
EXT_URL="https://github.com/PlotJuggler/pj-official-plugins/releases/download/data_stream_ros2%2Fv${EXT_VERSION}/ros2-topic-subscriber-${EXT_VERSION}-linux-x86_64.zip"
echo "Downloading ${EXT_URL}"
if ! fetch "${EXT_URL}" "${WORK}/ext.zip"; then
    note "ROS 2 live-topic extension not installed: cannot download ${EXT_URL} (bag files still open fine)"
    exit 0
fi
if ! verify "${WORK}/ext.zip" "${EXT_SHA256}"; then
    note "ROS 2 live-topic extension not installed: checksum mismatch for ${EXT_VERSION} (expected ${EXT_SHA256})"
    exit 0
fi
if ! python3 -m zipfile -e "${WORK}/ext.zip" "${WORK}/ext" || [ ! -f "${WORK}/ext/ros2-topic-subscriber/manifest.json" ]; then
    note "ROS 2 live-topic extension not installed: unexpected archive layout"
    exit 0
fi
mkdir -p "${EXT_DIR}"
rm -rf "${EXT_DIR}/ros2-topic-subscriber"
mv "${WORK}/ext/ros2-topic-subscriber" "${EXT_DIR}/ros2-topic-subscriber"
# python's zipfile does not keep the executable bit; the loader needs the shared objects to be readable only,
# but keep libraries executable like the upstream package does.
find "${EXT_DIR}/ros2-topic-subscriber" -type f \( -name '*.so' -o -name '*.pjros2' \) -exec chmod 755 {} +
chown -R dev:dev "${USER_HOME}/.local"

echo "PlotJuggler ${VERSION} and ROS 2 Topic Subscriber ${EXT_VERSION} installed."
