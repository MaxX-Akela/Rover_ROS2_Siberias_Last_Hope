# syntax=docker/dockerfile:1.7
# PX4 SITL (build) + standalone Gazebo server for the differential rover.
ARG TARGETPLATFORM

FROM ubuntu:24.04 AS px4-deps
ARG TARGETPLATFORM
ENV DEBIAN_FRONTEND=noninteractive

RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,id=apt-px4-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-px4-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential ccache cmake ninja-build git \
        python3 python3-dev python3-pip python3-setuptools python3-wheel \
        libxml2-dev libssl-dev libeigen3-dev libopencv-dev file rsync zip unzip wget gnupg \
        lsb-release ca-certificates

# Gazebo Harmonic
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

RUN --mount=type=cache,id=apt-px4-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-px4-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -qq && \
    apt-get install -y --no-install-recommends gz-harmonic

# ---------------------------------------------------------------- build PX4
FROM px4-deps AS px4-dev
ARG PX4_REPO=https://github.com/Pulse-Width-Modulation-Technologies/PX4-Autopilot-Rover.git
# Must match the px4_msgs version in third_party/rover.repos (release/1.16).
# The gz_rover_differential airframe (50000) needs a PX4 tree that contains it.
ARG PX4_VERSION=rover

WORKDIR /build
RUN git clone ${PX4_REPO} PX4-Autopilot
WORKDIR /build/PX4-Autopilot
RUN echo "Checkout PX4-Autopilot version: ${PX4_VERSION}" && \
    git switch --detach ${PX4_VERSION} && \
    git submodule update --init --recursive --depth 1
RUN pip3 install -r ./Tools/setup/requirements.txt --no-cache-dir --break-system-packages

RUN --mount=type=cache,id=ccache-px4-${TARGETPLATFORM},target=/root/.ccache \
    make px4_sitl_default

# collect gz plugins and check the airframe is present
RUN mkdir -p /out/plugins && \
    find build/px4_sitl_default/src/modules/simulation/gz_plugins -name '*.so' -exec cp {} /out/plugins/ \; && \
    ls ROMFS/px4fmu_common/init.d-posix/airframes/ | grep gz_rover_differential

# ---------------------------------------------------------------- PX4 SITL
FROM px4-deps AS px4-sitl
WORKDIR /px4
COPY --from=px4-dev /build/PX4-Autopilot/build/px4_sitl_default/bin ./bin
COPY --from=px4-dev /build/PX4-Autopilot/build/px4_sitl_default/etc ./etc
COPY --from=px4-dev /build/PX4-Autopilot/ROMFS ./ROMFS

# Gazebo runs in its own container; PX4 only spawns the model into it
ENV PX4_GZ_STANDALONE=1
ENV PX4_GZ_WORLD=default
ENV PX4_SYS_AUTOSTART=50000
ENV PX4_SIM_MODEL=gz_rover_differential

ENTRYPOINT ["/px4/bin/px4", "ROMFS/px4fmu_common", "-d", "/px4/etc"]

# ---------------------------------------------------------------- Gazebo server
FROM px4-deps AS gz
WORKDIR /gz
COPY --from=px4-dev /build/PX4-Autopilot/Tools/simulation/gz/models ./models
COPY --from=px4-dev /build/PX4-Autopilot/Tools/simulation/gz/worlds ./worlds
COPY --from=px4-dev /build/PX4-Autopilot/Tools/simulation/gz/server.config ./server.config
COPY --from=px4-dev /out/plugins ./plugins

ENV GZ_SIM_RESOURCE_PATH=/gz/models:/gz/worlds
ENV GZ_SIM_SERVER_CONFIG_PATH=/gz/server.config
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/gz/plugins
ENV GZ_IP=127.0.0.1

# headless server; connect a GUI with `gz sim -g` on the host (same network)
CMD ["gz", "sim", "-r", "-s", "/gz/worlds/default.sdf"]
