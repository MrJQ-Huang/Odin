#!/usr/bin/env bash
# Run with the Odin driver already publishing. Usage: bash record_odin.sh NEW_BAG_DIR
set -eo pipefail
if [[ $# != 1 ]]; then
  echo 'Usage: bash record_odin.sh NEW_BAG_DIR' >&2
  exit 2
fi
probe_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
exec ros2 bag record -s mcap -b 134217728 --max-cache-size 268435456 \
  --qos-profile-overrides-path "$probe_dir/rosbag2_qos.yaml" -o "$1" \
  /odin1/image/compressed /odin1/cloud_raw /odin1/imu \
  /odin1/odometry /odin1/odometry_highfreq /odin1/wiwc
