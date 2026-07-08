#!/usr/bin/env python3
"""Launch RViz with the K1 MUSE Pi Pro display config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("k1muse_slam_nav"),
        "rviz",
        "k1muse.rviz",
    )

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        )
    ])
