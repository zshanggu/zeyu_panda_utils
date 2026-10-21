#!/usr/bin/env bash
# Starts the panda-deoxys container with host networking (ZMQ to the NUC, ROS 2 topics) and the
# Deformable workspace mounted at the same path. Extra args are run as the command, e.g.
#   docker/run.sh python3 scripts/franka_manipulation.py --check
set -euo pipefail

WORKSPACE=/data/zeyu/Deformable
TTY_FLAGS=()
[ -t 0 ] && TTY_FLAGS=(-it)

# Pass through the SpaceMouse (any 3Dconnexion / Logitech-3Dconnexion HID) if plugged in; the
# hidraw number changes on replug, so look it up each time.
DEVICE_FLAGS=()
for dev in /sys/class/hidraw/hidraw*; do
    if grep -q -i -E 'HID_ID=0003:0000(046D:0000C6|256F:0000C6)' "$dev/device/uevent" 2>/dev/null; then
        DEVICE_FLAGS+=(--device "/dev/$(basename "$dev")")
    fi
done
# The pip hidapi wheel opens devices through libusb, so the raw USB node is needed too.
for dev in /sys/bus/usb/devices/*; do
    vid=$(cat "$dev/idVendor" 2>/dev/null) || continue
    pid=$(cat "$dev/idProduct" 2>/dev/null) || continue
    if [[ "$vid:$pid" =~ ^(046d|256f):c6 ]]; then
        DEVICE_FLAGS+=(--device "$(printf '/dev/bus/usb/%03d/%03d' "$(cat "$dev/busnum")" "$(cat "$dev/devnum")")")
    fi
done

exec docker run --rm "${TTY_FLAGS[@]}" "${DEVICE_FLAGS[@]}" -v /run/udev:/run/udev:ro \
    --network host --ipc host \
    --user "$(id -u):$(id -g)" --group-add plugdev \
    -e HOME=/tmp -e DISPLAY="${DISPLAY:-}" -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/panda_utils/docker/fastdds_udp_only.xml" \
    -v "$WORKSPACE:$WORKSPACE" \
    panda-deoxys "${@:-bash}"
