#!/usr/bin/env bash
set -euo pipefail
if ! docker info >/dev/null 2>&1; then
  exec sg docker -c "$(printf '%q ' "$0" "$@")"
fi
TASK_ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
TASK_IMAGE=${ODIN_WIRELESS_IMAGE:-kimera-odin1:wireless}
TASK_ACTION=${1:-help}
shift || true
case "$TASK_ACTION" in
  master)
    : "${ROS_IP:?Set ROS_IP to this PC LAN address}"
    exec docker run --rm --name odin-wireless-master --network host \
      -e ROS_IP -e ROS_MASTER_URI="http://${ROS_IP}:11311" "$TASK_IMAGE" roscore
    ;;
  robot)
    : "${ROS_IP:?Set ROS_IP to this PC LAN address}"
    : "${ROS_MASTER_URI:?Set ROS_MASTER_URI to the master LAN address}"
    TASK_NAME=${1:?odin_a or odin_b}; TASK_ID=${2:?robot ID 0 or 1}
    shift 2
    TASK_CONFIG="artifacts/wireless/${TASK_NAME}"
    read -r TASK_BUS TASK_ADDR < <(python3 "${TASK_ROOT}/docker/find_odin_usb.py" "${ODIN_TARGET_USB_SERIAL:?Set serial from USB sysfs}")
    test -f "${TASK_ROOT}/${TASK_CONFIG}/adapter.yaml"
    mkdir -p "${TASK_ROOT}/${TASK_CONFIG}/runtime/calibration" \
      "${TASK_ROOT}/${TASK_CONFIG}/runtime/lcd" "${TASK_ROOT}/${TASK_CONFIG}/runtime/dpgo"
    exec docker run --rm --name "odin-wireless-${TASK_NAME}" --network host \
      --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb \
      -v /run/udev:/run/udev:ro -v "${TASK_ROOT}:/workspace" \
      -e ROS_IP -e ROS_MASTER_URI -e ODIN_USB_BUS="$TASK_BUS" -e ODIN_USB_ADDR="$TASK_ADDR" \
      -e LD_PRELOAD=/opt/kimera_ws/devel/lib/liblibusb_odin_filter.so \
      "$TASK_IMAGE" roslaunch /workspace/launch/odin1_wireless_robot.launch \
      robot_name:="$TASK_NAME" robot_id:="$TASK_ID" \
      device_config:="/workspace/${TASK_CONFIG}/control_command.yaml" \
      adapter_config:="/workspace/${TASK_CONFIG}/adapter.yaml" \
      params_folder:="/workspace/${TASK_CONFIG}/params" \
      runtime_dir:="/workspace/${TASK_CONFIG}/runtime" "$@"
    ;;
  shell)
    exec docker run --rm -it --network host -v "${TASK_ROOT}:/workspace" \
      -e ROS_IP -e ROS_MASTER_URI "$TASK_IMAGE" bash
    ;;
  *) echo 'Set ROS_IP and ROS_MASTER_URI, then run: master | robot odin_a 0 | shell';;
esac
