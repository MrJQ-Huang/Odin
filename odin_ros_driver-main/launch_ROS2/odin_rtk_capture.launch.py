from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/odin_b/odin1/cloud_render"),
        DeclareLaunchArgument("fix_topic", default_value="/gnss/fix"),
        DeclareLaunchArgument("heading_topic", default_value="/gnss/heading"),
        DeclareLaunchArgument("status_topic", default_value="/gnss/status"),
        DeclareLaunchArgument("meta_topic", default_value="/odin_rtk/bound_meta"),
        DeclareLaunchArgument("bound_cloud_topic", default_value="/odin_rtk/bound_cloud"),
        DeclareLaunchArgument("republish_cloud", default_value="true"),
        Node(
            package="odin_ros_driver",
            executable="odin_rtk_capture_node.py",
            name="odin_rtk_capture_node",
            output="screen",
            parameters=[{
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "fix_topic": LaunchConfiguration("fix_topic"),
                "heading_topic": LaunchConfiguration("heading_topic"),
                "status_topic": LaunchConfiguration("status_topic"),
                "meta_topic": LaunchConfiguration("meta_topic"),
                "bound_cloud_topic": LaunchConfiguration("bound_cloud_topic"),
                "republish_cloud": ParameterValue(LaunchConfiguration("republish_cloud"), value_type=bool),
            }],
        ),
    ])
