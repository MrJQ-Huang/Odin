# Odin Dual Device Workspace

This repository contains the local Odin1 ROS2 workspace, the Manifold Odin wiki snapshot, and a local Web UI for managing two Odin1 devices on one host.

Main paths:

- `odin_ros_driver-main/`: Odin ROS driver with local dual-device adaptations.
- `ros2_ws/`: ROS2 workspace scaffold. `src/odin_ros_driver` is a relative symlink to `../../odin_ros_driver-main`.
- `wiki-master/`: downloaded Manifold wiki snapshot.
- `web_ui/`: local browser UI for USB binding, YAML editing, and dual launch control.
- `calib/`: captured A/B calibration snapshots from `~/.ros/odin_a` and `~/.ros/odin_b`.

Run the Web UI:

```bash
cd ~/odin/web_ui
python3 server.py --host 127.0.0.1 --port 8765
```

Run the dual Odin ROS2 launch directly:

```bash
cd ~/odin/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch odin_ros_driver dual_odin.launch.py
```

The Web UI stores Odin serial numbers and resolves the current USB bus/address before launch, because Linux USB device addresses can change after replugging.
