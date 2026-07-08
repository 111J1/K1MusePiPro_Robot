"""
robot.real_motion.launch.py — P1 minimum real-motion bringup.

Launches: TaskManager + ControlManager (real) + AI/voice/vision core.
Requires separately: k1muse_communicate_ros (mcu_bridge) and
k1muse_slam_ros (chassis_adapter, LD06).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_enabled = LaunchConfiguration("motion_enabled")
    params_file_ctrl = LaunchConfiguration("params_file_ctrl")
    params_file_task = LaunchConfiguration("params_file_task")

    return LaunchDescription([
        DeclareLaunchArgument("motion_enabled", default_value="true"),
        DeclareLaunchArgument(
            "params_file_ctrl",
            default_value=PathJoinSubstitution([
                FindPackageShare("k1muse_control_manager"),
                "config", "control_manager.real.yaml"])),
        DeclareLaunchArgument(
            "params_file_task",
            default_value=PathJoinSubstitution([
                FindPackageShare("k1muse_task_manager"),
                "config", "task_manager.real.yaml"])),

        # ── Core AI/Voice/Vision (from k1muse_core_ros) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("k1muse_robot_bringup"),
                "launch", "robot.mock.launch.py"]),
            launch_arguments={
                # keep mock devices for audio/camera simulation
            }.items(),
        ),

        # ── ControlManager (real motion) ──
        Node(
            package="k1muse_control_manager",
            executable="control_manager_node",
            name="control_manager_node",
            output="screen",
            parameters=[params_file_ctrl],
        ),

        # ── TaskManager (real) ──
        Node(
            package="k1muse_task_manager",
            executable="task_manager_node",
            name="task_manager_node",
            output="screen",
            parameters=[params_file_task],
        ),
    ])
