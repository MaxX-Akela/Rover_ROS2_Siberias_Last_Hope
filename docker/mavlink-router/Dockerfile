# syntax=docker/dockerfile:1.7
# mavlink-router (third-party, https://github.com/mavlink-router/mavlink-router) built as is.
# Not packaged in apt for Ubuntu 24.04, hence built from source.
ARG TARGETPLATFORM

FROM ubuntu:24.04 AS build
ARG TARGETPLATFORM
ARG MAVLINK_ROUTER_VERSION=v4
ENV DEBIAN_FRONTEND=noninteractive

RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

RUN --mount=type=cache,id=apt-mavr-${TARGETPLATFORM},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=apt-mavr-lists-${TARGETPLATFORM},target=/var/lib/apt/lists,sharing=locked \
    apt-get update -y && \
    apt-get install -y --no-install-recommends \
        git ca-certificates build-essential meson ninja-build pkg-config

RUN git clone --depth 1 --branch ${MAVLINK_ROUTER_VERSION} --recurse-submodules --shallow-submodules \
        https://github.com/mavlink-router/mavlink-router.git /src
WORKDIR /src
RUN meson setup build . --buildtype=release --prefix=/opt/mavlink-router -Dsystemdsystemunitdir=/opt/mavlink-router/lib/systemd && \
    ninja -C build install

FROM ubuntu:24.04
COPY --from=build /opt/mavlink-router/bin/mavlink-routerd /usr/local/bin/mavlink-routerd

# the config is mounted by docker/compose.yaml
CMD ["mavlink-routerd", "-c", "/etc/mavlink-router/main.conf"]
