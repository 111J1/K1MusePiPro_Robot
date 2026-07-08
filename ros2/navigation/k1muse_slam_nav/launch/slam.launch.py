#!/usr/bin/env python3
"""Launch sensors, Cartographer, and optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _start_cartographer(context, *args, **kwargs):
    del args, kwargs

    pkg_share = get_package_share_directory("k1muse_slam_nav")
    config_dir = os.path.join(pkg_share, "config")
    use_imu = _as_bool(LaunchConfiguration("enable_imu").perform(context))
    start_sensors = _as_bool(LaunchConfiguration("start_sensors").perform(context))

    config_basename = "cartographer_2d.lua"
    messages = []

    if use_imu:
        if not start_sensors or os.path.exists("/dev/myimu"):
            config_basename = "cartographer_2d_imu.lua"
            messages.append(LogInfo(msg="Cartographer using IMU input from /imu/data"))
        else:
            messages.append(LogInfo(
                msg="WARNING: enable_imu:=true but /dev/myimu is missing; falling back to Cartographer without IMU."
            ))
    else:
        messages.append(LogInfo(msg="Cartographer starting without IMU input"))

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory", config_dir,
            "-configuration_basename", config_basename,
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odom"),
            ("imu", "/imu/data"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0",
        ],
    )

    return messages + [cartographer_node, occupancy_grid_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("k1muse_slam_nav")
    sensors_launch = os.path.join(pkg_share, "launch", "_sensors.launch.py")
    rviz_config = os.path.join(pkg_share, "rviz", "k1muse.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    enable_imu = LaunchConfiguration("enable_imu")
    require_odom_precheck = LaunchConfiguration("require_odom_precheck")
    start_sensors = LaunchConfiguration("start_sensors")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "enable_cmd_vel_output",
            default_value="false",
            description="Compatibility only. /cmd_vel routing is now a mcu_bridge_node parameter.",
        ),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("require_odom_precheck", default_value="true"),
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
        TimerAction(period=1.0, actions=[
            OpaqueFunction(function=_start_cartographer),
        ]),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
