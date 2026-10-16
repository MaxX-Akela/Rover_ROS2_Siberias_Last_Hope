#!/usr/bin/env bash
# Builds the Raspberry Pi SD card image. Needs root (losetup, mount, chroot) on an arm64 host.
#
# Inputs (environment):
#   BASE_IMG_XZ    Ubuntu preinstalled server image (.img.xz), already verified
#   OUT_IMG        output .img path
#   IMAGES_DIR     directory with the *.tar.zst Docker image archives (optional)
#   ROVER_VERSION  release version, written to /opt/rover/.env as TAG
#   IMAGE_REGISTRY registry prefix of the images, e.g. ghcr.io/owner
#   EXTRA_GB       how much to grow the image for the build (default 8)
#   ROVER_IP, ROVER_PREFIX, ROVER_GW   static Ethernet address (defaults: 192.168.1.10, 24, 192.168.1.1)
#   ROVER_USER, ROVER_PASSWORD, ROVER_HOSTNAME   (defaults: rover, rover, rover)
set -euxo pipefail

: "${BASE_IMG_XZ:?}" "${OUT_IMG:?}" "${ROVER_VERSION:?}" "${IMAGE_REGISTRY:?}"
IMAGES_DIR="${IMAGES_DIR:-}"
EXTRA_GB="${EXTRA_GB:-8}"
export ROVER_IP="${ROVER_IP:-192.168.1.10}"
export ROVER_PREFIX="${ROVER_PREFIX:-24}"
export ROVER_GW="${ROVER_GW:-192.168.1.1}"
export ROVER_USER="${ROVER_USER:-rover}"
export ROVER_PASSWORD="${ROVER_PASSWORD:-rover}"
export ROVER_HOSTNAME="${ROVER_HOSTNAME:-rover}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PI="${REPO}/deploy/pi"
MNT="$(mktemp -d)"
LOOP=""

cleanup() {
    set +e
    for m in dev/pts dev proc sys boot/firmware ""; do
        mountpoint -q "${MNT}/${m}" && umount -l "${MNT}/${m}"
    done
    [[ -n "${LOOP}" ]] && losetup -d "${LOOP}"
}
trap cleanup EXIT

# ---- unpack, grow ---------------------------------------------------------------------
xz -dc "${BASE_IMG_XZ}" > "${OUT_IMG}"
truncate -s "+${EXTRA_GB}G" "${OUT_IMG}"
parted -s "${OUT_IMG}" resizepart 2 100%
LOOP="$(losetup -fP --show "${OUT_IMG}")"
e2fsck -fy "${LOOP}p2" || [[ $? -le 1 ]]
resize2fs "${LOOP}p2"

mount "${LOOP}p2" "${MNT}"
mount "${LOOP}p1" "${MNT}/boot/firmware"

# ---- /opt/rover -----------------------------------------------------------------------
ROVER="${MNT}/opt/rover"
mkdir -p "${ROVER}/docker/mavlink-router" "${ROVER}/rover_bringup" "${ROVER}/images"
cp "${REPO}/docker/compose.yaml" "${ROVER}/docker/compose.yaml"
cp "${REPO}/docker/mavlink-router/main.conf" "${ROVER}/docker/mavlink-router/main.conf"
cp -r "${REPO}/rover_bringup/config" "${ROVER}/rover_bringup/config"
install -m 0755 "${PI}/load-images.sh" "${ROVER}/load-images.sh"
cat > "${ROVER}/.env" <<ENV
TAG=${ROVER_VERSION}
ROS_IMAGE=${IMAGE_REGISTRY}/rover-ros:${ROVER_VERSION}
MAVLINK_ROUTER_IMAGE=${IMAGE_REGISTRY}/rover-mavlink-router:${ROVER_VERSION}
# uXRCE-DDS agent (Pixracer TELEM2)
XRCE_DEV=/dev/rover_fcu_telem
XRCE_BAUD=921600
ENV
if [[ -n "${IMAGES_DIR}" ]]; then
    cp "${IMAGES_DIR}"/*.tar.zst "${ROVER}/images/"
fi

install -m 0644 "${PI}/rover.service" "${PI}/rover-images-load.service" "${MNT}/etc/systemd/system/"
install -m 0644 "${PI}/99-rover.rules" "${MNT}/etc/udev/rules.d/99-rover.rules"

# ---- boot partition: cloud-init, network, config.txt ----------------------------------
BOOT="${MNT}/boot/firmware"
install -m 0644 "${PI}/user-data" "${BOOT}/user-data"
sed -e "s|@ROVER_IP@|${ROVER_IP}|g" -e "s|@ROVER_PREFIX@|${ROVER_PREFIX}|g" \
    -e "s|@ROVER_GW@|${ROVER_GW}|g" "${PI}/network-config.in" > "${BOOT}/network-config"

cat >> "${BOOT}/config.txt" <<'CFG'

# --- rover ---
[all]
enable_uart=1
camera_auto_detect=1

[pi4]
dtoverlay=disable-bt
[all]
CFG
# the UART carries uXRCE-DDS, not a login console
sed -i -E 's/ ?console=(serial0|ttyAMA0|ttyS0),[0-9]+//g' "${BOOT}/cmdline.txt"

# ---- chroot ---------------------------------------------------------------------------
mount --bind /dev "${MNT}/dev"
mount --bind /dev/pts "${MNT}/dev/pts"
mount -t proc proc "${MNT}/proc"
mount -t sysfs sys "${MNT}/sys"
rm -f "${MNT}/etc/resolv.conf"
printf 'nameserver 1.1.1.1\nnameserver 8.8.8.8\n' > "${MNT}/etc/resolv.conf"

install -m 0755 "${PI}/customize.sh" "${MNT}/tmp/customize.sh"
chroot "${MNT}" /usr/bin/env \
    ROVER_USER="${ROVER_USER}" ROVER_PASSWORD="${ROVER_PASSWORD}" ROVER_HOSTNAME="${ROVER_HOSTNAME}" \
    /bin/bash /tmp/customize.sh

rm -f "${MNT}/etc/resolv.conf"
ln -s ../run/systemd/resolve/stub-resolv.conf "${MNT}/etc/resolv.conf"
sync

# ---- unmount, shrink ------------------------------------------------------------------
umount -l "${MNT}/dev/pts" "${MNT}/dev" "${MNT}/proc" "${MNT}/sys"
umount "${MNT}/boot/firmware"
umount "${MNT}"

zerofree -v "${LOOP}p2"
e2fsck -fy "${LOOP}p2" || [[ $? -le 1 ]]
resize2fs -M "${LOOP}p2"
BLOCKS="$(dumpe2fs -h "${LOOP}p2" 2>/dev/null | awk -F: '/^Block count/ {gsub(/ /,"",$2); print $2}')"
BSIZE="$(dumpe2fs -h "${LOOP}p2" 2>/dev/null | awk -F: '/^Block size/ {gsub(/ /,"",$2); print $2}')"
START="$(partx -g -o START -n 2 "${LOOP}" | tr -d ' ')"
losetup -d "${LOOP}"
LOOP=""

SECTORS=$(( (BLOCKS * BSIZE + 511) / 512 ))
echo ", ${SECTORS}" | sfdisk --no-reread -N 2 "${OUT_IMG}"
truncate -s $(( (START + SECTORS) * 512 )) "${OUT_IMG}"
ls -l "${OUT_IMG}"
