#!/usr/bin/env bash
# Start the ZED cameras as ROS 2 topics from the zed-ros2 container (docker/zed.Dockerfile). Ctrl+C stops them.
# Replaces the lab's zed_tools/launch_all.sh (another Linux user's): same topics, /zed_multi/<camera>/...
#
#   docker/zed_cameras.sh                       # zed2i + zedm + zed
#   ZED_CAMERAS="zed2i:zed2i:33386243 zedm:zedm:15137730" docker/zed_cameras.sh   # a subset
#   ZED_DEPTH_MODE=NEURAL docker/zed_cameras.sh  # better depth (first start builds the AI model: minutes)
#   docker/zed_cameras.sh --list                # list connected cameras and their serial numbers, then exit
#
# Only one program can open a camera: stop any other ZED launcher first (this script refuses to start if
# /zed_multi topics are already being published).
#
# Env: ZED_CAMERAS ("name:model:serial ..."), ZED_DEPTH_MODE (PERFORMANCE | QUALITY | ULTRA | NEURAL | NONE),
#      ZED_CACHE (/data/zeyu/Deformable/.cache/zed: the SDK's per-camera calibration files, downloaded once).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$HERE/../.." && pwd)"
ZED_CAMERAS="${ZED_CAMERAS:-zed2i:zed2i:33386243 zedm:zedm:15137730 zed:zed:18838}"
ZED_DEPTH_MODE="${ZED_DEPTH_MODE:-PERFORMANCE}"
ZED_CACHE="${ZED_CACHE:-$WORKSPACE/.cache/zed}"
NAME="zed-cameras-$USER"

docker_cmd() { if docker info >/dev/null 2>&1; then docker "$@"; else sg docker -c "docker $(printf '%q ' "$@")"; fi; }

mkdir -p "$ZED_CACHE/settings"
COMMON=(--rm --gpus all --privileged -v /dev:/dev --network host --ipc host
        -e FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_udp_only.xml" -v "$HERE:$HERE:ro"
        -v "$ZED_CACHE:$ZED_CACHE" -v "$ZED_CACHE/settings:/usr/local/zed/settings")

if [[ "${1:-}" == "--list" ]]; then
    docker_cmd run "${COMMON[@]}" zed-ros2 /usr/local/zed/tools/ZED_Explorer --all
    exit $?
fi

if ! docker_cmd image inspect zed-ros2 >/dev/null 2>&1; then
    docker_cmd build -t zed-ros2 -f "$HERE/zed.Dockerfile" "$HERE"
fi

# Refuse to fight another launcher for the cameras.
existing="$(docker_cmd run --rm --network host --ipc host -e FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_udp_only.xml" \
    -v "$HERE:$HERE:ro" zed-ros2 bash -c 'source /opt/ros/humble/setup.bash; timeout 5 ros2 topic list 2>/dev/null' \
    | grep -c '^/zed_multi/' || true)"
if (( existing > 0 )); then
    echo "error: /zed_multi topics are already published (another ZED launcher, e.g. res1's launch_all.sh, is running)." >&2
    echo "       Only one program can open the cameras; stop that one first." >&2
    exit 1
fi

# The launch file only takes a parameter file, so bake the depth mode into a per-run copy.
PARAMS="$ZED_CACHE/zed_params_run.yaml"
sed -E "s/^(\s*depth_mode:\s*)'[A-Z_]+'/\1'$ZED_DEPTH_MODE'/" "$HERE/zed_params.yaml" > "$PARAMS"

# All cameras go into ONE component container (one process), as the lab's launcher did (zed_container_0 only).
# The ZED SDK cannot find a camera from a second process while another process has cameras open ("CAMERA NOT
# DETECTED"), and with the launch file's own container all three would also clash on its default name. When
# container_name is given the launch file loads into an existing container instead of creating one, so it is
# started here first. Cameras are loaded one after another. publish_urdf must stay on: the wrapper waits for the
# camera's static TFs (from its URDF) before it grabs, even with positional tracking off.
launches="ros2 run rclcpp_components component_container_isolated --use_multi_threaded_executor \
--ros-args -r __node:=zed_container_0 -r __ns:=/zed_multi & sleep 3; "
for spec in $ZED_CAMERAS; do
    IFS=: read -r cam model serial <<< "$spec"
    if [[ "$serial" == ZED_SERIAL ]]; then
        echo "error: the serial number of camera '$cam' is not set yet: run 'docker/zed_cameras.sh --list' and put it in" >&2
        echo "       ZED_CAMERAS, or start without it, e.g. ZED_CAMERAS=\"zed2i:zed2i:33386243 zedm:zedm:15137730\"" >&2
        exit 1
    fi
    launches+="ros2 launch zed_wrapper zed_camera.launch.py camera_name:=$cam camera_model:=$model namespace:=zed_multi \
container_name:=zed_container_0 \
serial_number:=$serial publish_tf:=false publish_map_tf:=false publish_imu_tf:=false publish_urdf:=true \
ros_params_override_path:=$PARAMS & sleep 8; "
done

# A previous run that is still shutting down keeps the cameras busy: wait until it is gone.
for _ in $(seq 1 30); do docker_cmd ps -q -f name="^$NAME$" | grep -q . || break; sleep 1; done
echo "Starting ZED cameras: $ZED_CAMERAS (depth $ZED_DEPTH_MODE). Topics: /zed_multi/<name>/rgb/image_rect_color, ..."
# Shut down gracefully: SIGKILLing the ZED driver mid-stream once left the ZED 2i undetectable until it was
# replugged. --init forwards docker stop's SIGTERM to the inner shell, which passes SIGTERM on to ros2 launch / the
# component container (background jobs of a non-interactive shell ignore SIGINT) so the driver closes the
# cameras. The container runs detached and its log is followed, so
# Ctrl+C here always reaches the stop below.
stop_cameras() {
    trap - EXIT INT TERM
    echo "Stopping ZED cameras (closing them cleanly takes a few seconds)..."
    docker_cmd stop -t 20 "$NAME" >/dev/null 2>&1 || true
}
trap stop_cameras EXIT INT TERM
docker_cmd run -d --init "${COMMON[@]}" --name "$NAME" zed-ros2 bash -c \
    "source /opt/ros/humble/setup.bash; source /ros2_ws/install/local_setup.bash; \
trap 'kill -TERM \$(jobs -p) 2>/dev/null; wait' INT TERM; $launches wait" >/dev/null
docker_cmd logs -f "$NAME" &
wait $!
