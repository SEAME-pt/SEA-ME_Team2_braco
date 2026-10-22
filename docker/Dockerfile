# SEA:ME 2.0 - arm class images, one Dockerfile with three stages:
#
#   base           Ubuntu 26.04 + ROS 2 Lyrical Luth (ros-base) + ros2_control + build/test tooling + pymycobot
#   jetson         base + serial tooling - the aarch64 robot-side image for the Jetson Nano (braco-image/jetson)
#   pc             base + RViz2, MoveIt 2, Gazebo (ros_gz), QA tooling - the x86_64 workstation image (braco-image/pc)
#
# Both final images are built FROM the same `base` stage, so the ROS distro, ros2_control and pymycobot
# versions cannot drift apart between the two machines. Stages (instead of three files linked by a
# build context) are deliberate: with a `service:` build context Compose rebuilds on every
# `docker compose up`, and prebuilt images cannot be pulled on the Jetson. With stages, `up` only builds
# when an image is missing and BuildKit shares the base layers by itself.
#
#   docker build --target pc     -t braco-image/pc:lyrical     docker
#   docker build --target jetson -t braco-image/jetson:lyrical docker
#
# The `base` steps follow "Installing on Ubuntu - how-to" from the ROS 2 Lyrical documentation:
# locale, Universe, ros2-apt-source, `apt upgrade`, then the ROS packages.
#
# No `# syntax=` directive on purpose: it makes BuildKit pull a frontend image from Docker Hub on every
# build (one more network dependency and rate-limit exposure). `COPY --chmod` is built into BuildKit's
# default frontend, but the legacy builder rejects it: the host needs the buildx plugin (make doctor checks).

ARG UBUNTU_VERSION=26.04

# ============================================================================================
# base
# ============================================================================================
FROM ubuntu:${UBUNTU_VERSION} AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=lyrical
# Pinned so the build never depends on GitHub's rate-limited API. Use `latest` to resolve dynamically.
ARG ROS_APT_SOURCE_VERSION=1.3.0
# Leave empty for the latest pymycobot from PyPI; pin it once the class settles on a version.
ARG PYMYCOBOT_VERSION=
ARG USER_UID=1000
ARG USER_GID=1000

ENV ROS_DISTRO=${ROS_DISTRO} \
    SEAME_WS=/ws \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1 \
    COLCON_DEFAULTS_FILE=/etc/seame/colcon-defaults.yaml \
    RCUTILS_COLORIZED_OUTPUT=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- 1. Set up the system: UTF-8 locale and the Universe repository (as in the ROS docs) -------
RUN apt-get update \
 && apt-get install -y --no-install-recommends locales software-properties-common ca-certificates curl \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository -y universe \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# --- 2. ROS 2 apt repository (ros2-apt-source) --------------------------------------------------
COPY --chmod=0755 scripts/install-ros-apt-source.sh /usr/local/bin/install-ros-apt-source
RUN install-ros-apt-source "${ROS_APT_SOURCE_VERSION}"

# --- 3. "Ensure your system is up to date" - the ROS packages are built against current Ubuntu --
#        libraries, so skipping this on a container image that predates them gives unmet dependencies.
RUN apt-get update \
 && apt-get -y -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold upgrade \
 && rm -rf /var/lib/apt/lists/*

# --- 4. System tooling and pymycobot (Ubuntu + PyPI only: cheap, so any problem shows up early) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build ccache pkg-config git \
        wget sudo bash-completion less nano vim tmux htop \
        iproute2 iputils-ping net-tools openssh-client picocom \
        python3-pip python3-dev python3-numpy python3-yaml python3-serial \
        python3-fastapi python3-uvicorn python3-websockets \
 && rm -rf /var/lib/apt/lists/*
# (fastapi/uvicorn: the braco web page, served by the `real` container on the Jetson and by
#  the `sim` container on the PC, so both images need them)

# Same version on PC and Jetson because both inherit this layer. (pymycobot also pulls pyserial and
# numpy - already provided by apt above - and bleak.) It must live in the system Python so that
# ROS nodes can import it.
RUN pip3 install "pymycobot${PYMYCOBOT_VERSION:+==${PYMYCOBOT_VERSION}}" \
 && pip3 show pymycobot

# --- 5. ROS 2 core + ros2_control + test plumbing (names checked against the Lyrical rosdistro) --
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rclcpp-lifecycle \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-ros2controlcli \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-cmake-pytest \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-launch-testing-ros \
 && rm -rf /var/lib/apt/lists/*

# Helper used for packages whose binary build might be missing on one architecture.
COPY --chmod=0755 scripts/apt-install-optional.sh /usr/local/bin/apt-install-optional

RUN apt-get update \
 && apt-install-optional \
        ros-${ROS_DISTRO}-rmw-zenoh-cpp \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        ros-${ROS_DISTRO}-tf-transformations \
 && rm -rf /var/lib/apt/lists/*

# --- 6. Non-root user matching the host UID/GID, so bind-mounted ros2_ws stays writable --------
#     (Ubuntu 24.04+ images ship a default 'ubuntu' user at UID 1000; remove it first.)
RUN set -eux; \
    if getent passwd "${USER_UID}" >/dev/null; then userdel -r "$(getent passwd "${USER_UID}" | cut -d: -f1)" || true; fi; \
    if getent group "${USER_GID}" >/dev/null; then groupdel "$(getent group "${USER_GID}" | cut -d: -f1)" || true; fi; \
    groupadd --gid "${USER_GID}" dev; \
    useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash dev; \
    for g in dialout video render plugdev; do if getent group "$g" >/dev/null; then usermod -aG "$g" dev; fi; done; \
    echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/90-dev; \
    chmod 0440 /etc/sudoers.d/90-dev; \
    mkdir -p /ws/src /home/dev/.ccache /home/dev/.ros /home/dev/.gz; \
    chown -R dev:dev /ws /home/dev

# --- 7. rosdep + shared configuration -----------------------------------------------------------
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

COPY scripts/env.sh /usr/local/share/seame/env.sh
COPY config/colcon-defaults.yaml /etc/seame/colcon-defaults.yaml
COPY --chmod=0755 scripts/ros_entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY --chmod=0755 scripts/smoke-test.sh /usr/local/bin/seame-smoke-test

USER dev
WORKDIR /ws

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
 || echo "WARNING: rosdep update failed at build time; run 'rosdep update' once inside the container."

# Interactive shells (docker compose exec <svc> bash) get the same environment as the entrypoint.
RUN printf '%s\n' \
      'source /usr/local/share/seame/env.sh' \
      '[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' \
      'export PS1="\[\e[1;36m\](${SEAME_ROLE:-seame})\[\e[0m\] \u@\h:\w\$ "' \
      >> /home/dev/.bashrc

ENV SEAME_ROLE=base
ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]

# ============================================================================================
# jetson - robot side (Module 1: pymycobot lifecycle driver, ros2_control, controllers)
#   No GUI, MoveIt or Gazebo: those run on the PC and talk to this container over DDS.
#   GPU / TensorRT is intentionally NOT here - see README, "Module 2 and the Nano".
# ============================================================================================
FROM base AS jetson

ARG DEBIAN_FRONTEND=noninteractive
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV SEAME_ROLE=jetson-driver

# picocom/pyserial are already in base. libserial-dev is only needed if you implement the myCobot
# protocol in a C++ hardware_interface instead of calling pymycobot.
# demo_nodes_cpp / demo_nodes_py are the C++ talker and Python listener used by `make net-test`
# (ros-base does not include them; the PC image gets them from ros-lyrical-desktop). They are added in this
# stage, not in `base`, so the PC image's layers stay cached.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        libserial-dev \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-demo-nodes-py \
 && rm -rf /var/lib/apt/lists/*

USER dev
CMD ["bash"]

# ============================================================================================
# pc - workstation: RViz2, MoveIt 2, Gazebo + ros2_control in simulation, CI-grade QA tooling
# ============================================================================================
FROM base AS pc

ARG DEBIAN_FRONTEND=noninteractive
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV SEAME_ROLE=pc-dev \
    QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb

# --- 1. OpenGL / Vulkan / X11 runtime. Mesa software rendering works everywhere; GPUs are added
#        through the compose/dri.yml / compose/nvidia.yml overrides. -------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        fonts-dejavu-core \
        libgl1 libglx0 libegl1 libgles2 libglvnd0 \
        libgl1-mesa-dri libglx-mesa0 mesa-vulkan-drivers vulkan-tools \
        libxkbcommon-x11-0 libxcb-cursor0 \
        mesa-utils x11-apps \
 && rm -rf /var/lib/apt/lists/*

# --- 2. Quality gates (clang-tidy, cppcheck, coverage) and analysis libraries (Ubuntu + PyPI) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        clang clang-format clang-tidy cppcheck lcov gcovr \
        gdb valgrind graphviz \
        python3-pytest-cov python3-pytest-mock \
        python3-scipy python3-matplotlib python3-pandas \
        python3-venv \
 && rm -rf /var/lib/apt/lists/*

# ruff and pre-commit are standalone tools, so they get their own virtualenv. Installing them next to the
# apt-managed Python packages fails: pre-commit -> virtualenv wants packaging>=26.3, Ubuntu ships 26.0
# without a pip RECORD file, and pip refuses to uninstall it ("installed by debian").
RUN python3 -m venv /opt/dev-tools \
 && /opt/dev-tools/bin/pip install --no-cache-dir ruff pre-commit \
 && ln -s /opt/dev-tools/bin/ruff /usr/local/bin/ruff \
 && ln -s /opt/dev-tools/bin/pre-commit /usr/local/bin/pre-commit

# --- 3. RViz2 (desktop), Gazebo and MoveIt 2 - all released for Lyrical (checked in the rosdistro).
#        Gazebo is installed ONLY through ros_gz so its version is the one this ROS distro pairs
#        with (Jetty, from packages.ros.org); never install gz-* packages separately. ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-ros-gz \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-moveit \
 && rm -rf /var/lib/apt/lists/*

# --- 4. Extras: warn instead of failing if a binary is missing. -------------------------------
#        moveit_servo is NOT part of the `moveit` metapackage (checked in its package.xml), so it is
#        installed explicitly. (PlotJuggler is handled in step 5: the ROS apt package
#        ros-lyrical-plotjuggler-ros has no binary for 26.04.)
RUN apt-get update \
 && apt-install-optional \
        ros-${ROS_DISTRO}-moveit-servo \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-gz-ros2-control-demos \
        ros-${ROS_DISTRO}-moveit-resources \
        ros-${ROS_DISTRO}-moveit-visual-tools \
        ros-${ROS_DISTRO}-rqt-controller-manager \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-turtlesim \
        liburdfdom-tools \
 && rm -rf /var/lib/apt/lists/*

# --- 5. PlotJuggler 4 (github.com/PlotJuggler/PlotJuggler) -------------------------------------
#        The release .deb is self-contained (bundles Qt6) and runs on 26.04; extension 1.1.1 of its ROS 2
#        Topic Subscriber adds the `lyrical` build that the .deb's bundled 1.0.2 lacks. Both are pinned
#        by SHA-256 (see scripts/install-plotjuggler.sh to upgrade). amd64 only; never fails the build.
ARG PLOTJUGGLER_VERSION=4.0.0
ARG PLOTJUGGLER_DEB_SHA256=f02b012aaefbec2699a659776bc77ff5c6327f03eb87eeed5fcf76f9fa93a141
ARG PJ_ROS2_EXTENSION_VERSION=1.1.1
ARG PJ_ROS2_EXTENSION_SHA256=0ef63a8a7e46f4823c804ebd6e5c93b9a6bd5bbfbf88967e97eeac8ad44b1e96
COPY --chmod=0755 scripts/install-plotjuggler.sh /usr/local/bin/install-plotjuggler
RUN install-plotjuggler

USER dev
CMD ["bash"]
