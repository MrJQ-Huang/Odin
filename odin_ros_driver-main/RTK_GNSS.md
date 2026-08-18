# RTK GNSS serial node

This package includes a ROS2 node for Beitian BG-358 / UM982-style RTK GNSS
receivers connected through a USB serial bridge.

The RTK PDFs in `RTK/` describe:

- Default debug serial baudrate: `115200`.
- Type-C/debug port and UART output port are `COM1` on BG-358 style modules.
- Default useful output includes `GGA`, `RMC`, `HEADINGA`, and optionally `GSA`,
  `GSV`, `VTG`, `HDT`, `TRA`.
- `GGA` fix quality: `1` is single point, `2` is DGPS/SBAS, `4` is RTK fixed,
  `5` is RTK float.
- `HEADINGA` reports dual-antenna heading state. `INSUFFICIENT_OBS` means heading
  is not solved yet.

## Start

```bash
source /opt/ros/humble/setup.bash
source ~/odin/ros2_ws/install/setup.bash
ros2 launch odin_ros_driver gnss_rtk.launch.py port:=/dev/ttyACM0 baudrate:=115200
```

If the serial port is owned by `root:dialout`, either grant temporary access:

```bash
pkexec /bin/chmod a+rw /dev/ttyACM0
```

or add the user to `dialout` and log out/in:

```bash
sudo usermod -aG dialout "$USER"
```

## Topics

With the default namespace `gnss`, the node publishes:

- `/gnss/fix` (`sensor_msgs/NavSatFix`): position from `GGA`.
- `/gnss/heading` (`geometry_msgs/Vector3Stamped`): x=heading deg, y=pitch deg,
  z=baseline meters from `HEADINGA`, `HDT`, or `TRA`.
- `/gnss/heading_deg` (`std_msgs/Float64`): heading only.
- `/gnss/status` (`std_msgs/String`): JSON summary of parsed fix/heading state.
- `/gnss/raw_sentence` (`std_msgs/String`): raw serial sentences.

## Quick Checks

```bash
ros2 topic echo /gnss/status
ros2 topic echo /gnss/fix
ros2 topic echo /gnss/heading
ros2 topic hz /gnss/raw_sentence
```

Raw serial check without ROS:

```bash
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
timeout 10s cat /dev/ttyACM0
```
