# syntax=docker/dockerfile:1.7-labs
# Build context: repository root (see docker/compose.yaml)
ARG BASE_IMAGE=ros:jazzy-ros-base
ARG ROS_DISTRO=jazzy
ARG TARGETPLATFORM

# ---------------------------------------------------------------- base
FROM ${BASE_IMAGE} AS base
ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive
# rosdep installs pymavlink (rover_teleop) with pip; Ubuntu 24.04 marks system Python as externally managed
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# keep downloaded .deb files so the apt cache mounts are useful
RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,id=apt-ros-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-ros-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && \
    apt-get install -y --no-install-recommends \
        gosu \
        sudo \
        git \
        build-essential \
        cmake \
        python3-rosdep \
        python3-vcstool \
        python3-colcon-common-extensions \
        python3-pip

RUN rosdep init || true

# ---------------------------------------------------------------- Micro XRCE-DDS Agent
FROM base AS xrce-agent
ARG TARGETPLATFORM
ARG XRCE_AGENT_VERSION=v2.4.3

RUN git clone --depth 1 --branch ${XRCE_AGENT_VERSION} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /src/agent
WORKDIR /src/agent/build
RUN --mount=type=cache,id=apt-ros-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-ros-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && apt-get install -y --no-install-recommends libssl-dev && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce && \
    make -j"$(nproc)" && make install

# ---------------------------------------------------------------- runtime dependencies (rosdep)
FROM base AS deps
ARG ROS_DISTRO
ARG TARGETPLATFORM

WORKDIR /ws
COPY third_party/rover.repos /ws/rover.repos
RUN vcs import --shallow src < rover.repos

# GStreamer for rover_vision/video_streamer (python3-gi and the plugins also come from
# package.xml; the RTSP server typelib has no rosdep key)
RUN --mount=type=cache,id=apt-ros-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-ros-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && \
    apt-get install -y --no-install-recommends \
        python3-gi \
        gir1.2-gstreamer-1.0 \
        gir1.2-gst-rtsp-server-1.0 \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-ugly

# only package.xml files, so the dependency layer is cached until dependencies change
COPY --parents ./rover*/package.xml /ws/src/
RUN --mount=type=cache,id=apt-ros-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-ros-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    rosdep install --from-paths src --ignore-src -y -r \
        --rosdistro ${ROS_DISTRO} --dependency-types=exec

# ---------------------------------------------------------------- build
FROM deps AS builder
ARG ROS_DISTRO
ARG TARGETPLATFORM

RUN --mount=type=cache,id=apt-ros-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-ros-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && \
    rosdep install --from-paths src --ignore-src -y -r \
        --rosdistro ${ROS_DISTRO} --dependency-types=build --dependency-types=buildtool

COPY --parents ./rover*/ /ws/src/

RUN --mount=type=cache,id=ros-build-${TARGETPLATFORM},target=/ws/build \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --merge-install --install-base /opt/rover \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# ---------------------------------------------------------------- final image
FROM deps AS rover-ros
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

COPY --from=xrce-agent /opt/xrce /usr/local
COPY --from=builder /opt/rover /opt/rover
RUN ldconfig

COPY docker/ros/entrypoint.sh /entrypoint.sh

WORKDIR /opt/rover
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "rover_bringup", "bringup.launch.py"]
