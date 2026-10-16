#!/usr/bin/env bash
# Loads /opt/rover/images/*.tar.zst into Docker, then removes the archives.
set -euo pipefail
shopt -s nullglob
for archive in /opt/rover/images/*.tar.zst; do
    echo "loading ${archive}"
    zstd -dc "${archive}" | docker load
    rm -f "${archive}"
done
