"""Patch an isolated copy of the dual-ROS vendor driver for Noetic."""
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
(root / 'package.xml').write_text('''<?xml version="1.0"?>
<package format="2">
 <name>odin_ros_driver</name><version>0.0.1</version>
 <description>Odin SDK ROS1 driver, isolated build adaptation</description>
 <maintainer email="support@lyd.com">Odin driver maintainers</maintainer>
 <license>Apache-2.0</license><buildtool_depend>catkin</buildtool_depend>
 <build_depend>message_generation</build_depend><exec_depend>message_runtime</exec_depend>
 <depend>roscpp</depend><depend>std_msgs</depend><depend>sensor_msgs</depend>
 <depend>nav_msgs</depend><depend>cv_bridge</depend><depend>tf</depend>
 <depend>image_transport</depend><depend>visualization_msgs</depend>
 <depend>roslib</depend>
</package>
''')
p = root / 'src/host_sdk_sample.cpp'
s = p.read_text()
old = '    ros::NodeHandle nh;\n    g_ros_object = new MultiSensorPublisher(nh);'
new = '''    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("frame_prefix", g_frame_prefix, "");
    private_nh.param<std::string>("command_file", g_command_file_path, "/tmp/odin_command.txt");
    g_ros_object = new MultiSensorPublisher(nh);'''
assert old in s
s = s.replace(old, new, 1)
old = '''                config_dir = std::filesystem::path(config_file).parent_path().string();
            }
        #endif'''
new = '''                config_dir = std::filesystem::path(config_file).parent_path().string();
            }
        #else
            private_nh.param<std::string>("config_file", config_file, config_file);
            config_dir = std::filesystem::path(config_file).parent_path().string();
        #endif'''
assert old in s
p.write_text(s.replace(old, new, 1))
p = root / 'CMakeLists.txt'
s = p.read_text()
a = s.index('    add_library(pointcloud_depth_converter')
b = s.index('    # Installation rules', a)
s = s[:a] + '''    add_library(libusb_odin_filter SHARED src/libusb_odin_filter.c)
    target_link_libraries(libusb_odin_filter ${LIBUSB_LIBRARIES} dl)
    install(TARGETS libusb_odin_filter LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

''' + s[b:]
s = s.replace('        message_generation\n', '        message_generation\n        roslib\n        visualization_msgs\n')
p.write_text(s)
