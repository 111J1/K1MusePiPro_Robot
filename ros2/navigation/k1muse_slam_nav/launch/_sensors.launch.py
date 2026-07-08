#!/usr/bin/env python3
"""Launch hardware sensors and robot description."""

import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _check_mcu_bridge(context, *args, **kwargs):
    del args, kwargs
    if not _as_bool(LaunchConfiguration("require_odom_precheck").perform(context)):
        return [LogInfo(msg="/odom precheck skipped by launch argument")]

    deadline = time.monotonic() + 8.0
    last_error = ""
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            last_error = "ros2 topic list timed out"
            time.sleep(0.25)
            continue

        if result.returncode != 0:
            last_error = result.stderr.strip() or f"ros2 topic list rc={result.returncode}"

        topics = set(result.stdout.splitlines())
        if "/odom" in topics:
            return [LogInfo(msg="/odom precheck passed")]
        time.sleep(0.25)

    detail = f" Last error: {last_error}" if last_error else ""
    return [
        LogInfo(msg=f"No /odom topic visible within 8s. Start mcu_bridge_node first.{detail}"),
        Shutdown(reason="mcu_bridge_node odom topic is not ready"),
    ]


def generate_launch_description():
    enable_imu = LaunchConfiguration("enable_imu")

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("k1muse_description"),
            "urdf",
            "musepi_robot.urdf.xacro",
        ]),
    ])
    robot_description_value = ParameterValue(robot_description, value_type=str)

    precheck = OpaqueFunction(function=_check_mcu_bridge)

    lidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD06",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD06"},
            {"topic_name": "scan_raw"},
            {"frame_id": "base_laser"},
            {"port_name": "/dev/mylidar"},
            {"port_baudrate": 230400},
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
            {"scan_beam_count": 720},
        ],
    )

    scan_self_filter_node = Node(
        package="k1muse_slam_nav",
        executable="k1_scan_self_filter.py",
        name="k1_scan_self_filter",
        output="screen",
        parameters=[{
            "input_topic": "/scan_raw",
            "output_topic": "/scan",
            "target_frame": "base_footprint",
            "base_link_name": "base_link",
            "footprint_padding": 0.02,
            "fallback_half_x": 0.198,
            "fallback_half_y": 0.198,
            "robot_description": robot_description_value,
        }],
    )

    imu_node = Node(
        package="imu_ros2_device",
        executable="ybimu_driver",
        name="ybimu_node",
        output="screen",
        condition=IfCondition(enable_imu),
        parameters=[{
            "port": "/dev/myimu",
            "frame_id": "imu_link",
            "publish_rate": 10.0,
            "enable_imu_data": True,
            "enable_imu_raw": False,
            "enable_mag": False,
            "enable_baro": False,
            "enable_euler": False,
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_value}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_imu",
            default_value="true",
            description="Start USB IMU driver by default; Cartographer can fall back when no /imu/data is available.",
        ),
        DeclareLaunchArgument(
            "require_odom_precheck",
            default_value="true",
            description="Require /odom to be visible before starting sensors.",
        ),
        precheck,
        lidar_node,
        scan_self_filter_node,
        imu_node,
        robot_state_publisher,
    ])
