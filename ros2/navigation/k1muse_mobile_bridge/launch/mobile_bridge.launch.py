from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("k1muse_mobile_bridge"),
        "config",
        "mobile_bridge.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("rfcomm_device", default_value="/dev/rfcomm0"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("tile_size", default_value="64"),
        DeclareLaunchArgument("map_publish_hz", default_value="1.0"),
        DeclareLaunchArgument("pose_publish_hz", default_value="10.0"),
        DeclareLaunchArgument("path_publish_hz", default_value="1.0"),
        Node(
            package="k1muse_mobile_bridge",
            executable="mobile_bridge_node",
            name="mobile_bridge_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "rfcomm_device": LaunchConfiguration("rfcomm_device"),
                    "map_topic": LaunchConfiguration("map_topic"),
                    "global_frame": LaunchConfiguration("global_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "tile_size": ParameterValue(LaunchConfiguration("tile_size"), value_type=int),
                    "map_publish_hz": ParameterValue(
                        LaunchConfiguration("map_publish_hz"), value_type=float
                    ),
                    "pose_publish_hz": ParameterValue(
                        LaunchConfiguration("pose_publish_hz"), value_type=float
                    ),
                    "path_publish_hz": ParameterValue(
                        LaunchConfiguration("path_publish_hz"), value_type=float
                    ),
                },
            ],
        ),
    ])
