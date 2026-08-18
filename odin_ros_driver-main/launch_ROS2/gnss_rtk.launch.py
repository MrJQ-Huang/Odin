from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("frame_id", default_value="gnss_link"),
        DeclareLaunchArgument("namespace", default_value="gnss"),
        Node(
            package="odin_ros_driver",
            executable="gnss_rtk_node.py",
            namespace=LaunchConfiguration("namespace"),
            name="gnss_rtk_node",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
    ])
