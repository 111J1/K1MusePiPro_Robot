#!/usr/bin/env python3
"""Launch sensors, Nav2 bringup, and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("k1muse_slam_nav")
    nav2_share = get_package_share_directory("nav2_bringup")
    sensors_launch = os.path.join(pkg_share, "launch", "_sensors.launch.py")
    nav2_bringup = os.path.join(nav2_share, "launch", "bringup_launch.py")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "my_map.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "k1muse.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_imu = LaunchConfiguration("enable_imu")
    require_odom_precheck = LaunchConfiguration("require_odom_precheck")
    start_sensors = LaunchConfiguration("start_sensors")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument(
            "enable_cmd_vel_output",
            default_value="true",
            description="Compatibility only. /cmd_vel routing is now a mcu_bridge_node parameter.",
        ),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument(
            "require_odom_precheck",
            default_value="false",
            description="Skip slow ROS graph CLI precheck by default for Nav2 bringup.",
        ),
        DeclareLaunchArgument(
            "start_sensors",
            default_value="true",
            description="Start local hardware sensors. Set false when sensors are published by another host.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            launch_arguments={
                "enable_imu": enable_imu,
                "require_odom_precheck": require_odom_precheck,
            }.items(),
            condition=IfCondition(start_sensors),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "slam": "False",
                "use_composition": "False",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
