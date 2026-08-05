import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')
    package_prefix = get_package_prefix('odin_ros_driver')
    libusb_filter = os.path.join(package_prefix, 'lib', 'libusb_odin_filter.so')

    odin_a_serial = LaunchConfiguration('odin_a_serial')
    odin_b_serial = LaunchConfiguration('odin_b_serial')
    odin_a_usb_bus = LaunchConfiguration('odin_a_usb_bus')
    odin_a_usb_addr = LaunchConfiguration('odin_a_usb_addr')
    odin_b_usb_bus = LaunchConfiguration('odin_b_usb_bus')
    odin_b_usb_addr = LaunchConfiguration('odin_b_usb_addr')
    use_rviz = LaunchConfiguration('use_rviz')

    odin_a_config = LaunchConfiguration('odin_a_config')
    odin_b_config = LaunchConfiguration('odin_b_config')
    odin_a_calib_dir = LaunchConfiguration('odin_a_calib_dir')
    odin_b_calib_dir = LaunchConfiguration('odin_b_calib_dir')

    odin_a_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        namespace='odin_a',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': odin_a_config,
            'target_serial': odin_a_serial,
            'frame_prefix': 'odin_a',
            'command_file': '/tmp/odin_a_command.txt',
        }],
        additional_env={
            'ODIN_CALIB_DIR': odin_a_calib_dir,
            'ODIN_USB_BUS': odin_a_usb_bus,
            'ODIN_USB_ADDR': odin_a_usb_addr,
            'LD_PRELOAD': libusb_filter,
        },
    )

    odin_b_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        namespace='odin_b',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': odin_b_config,
            'target_serial': odin_b_serial,
            'frame_prefix': 'odin_b',
            'command_file': '/tmp/odin_b_command.txt',
        }],
        additional_env={
            'ODIN_CALIB_DIR': odin_b_calib_dir,
            'ODIN_USB_BUS': odin_b_usb_bus,
            'ODIN_USB_ADDR': odin_b_usb_addr,
            'LD_PRELOAD': libusb_filter,
        },
    )

    world_to_odin_a = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odin_a_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odin_a/odom'],
    )

    world_to_odin_b = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odin_b_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odin_b/odom'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('odin_a_serial', default_value=''),
        DeclareLaunchArgument('odin_b_serial', default_value=''),
        DeclareLaunchArgument('odin_a_usb_bus', default_value='2'),
        DeclareLaunchArgument('odin_a_usb_addr', default_value='3'),
        DeclareLaunchArgument('odin_b_usb_bus', default_value='2'),
        DeclareLaunchArgument('odin_b_usb_addr', default_value='4'),
        DeclareLaunchArgument(
            'odin_a_config',
            default_value=os.path.join(package_dir, 'config', 'control_command_odin_a.yaml'),
        ),
        DeclareLaunchArgument(
            'odin_b_config',
            default_value=os.path.join(package_dir, 'config', 'control_command_odin_b.yaml'),
        ),
        DeclareLaunchArgument('odin_a_calib_dir', default_value='/home/uros/.ros/odin_a'),
        DeclareLaunchArgument('odin_b_calib_dir', default_value='/home/uros/.ros/odin_b'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        world_to_odin_a,
        world_to_odin_b,
        odin_a_node,
        odin_b_node,
        rviz_node,
    ])
