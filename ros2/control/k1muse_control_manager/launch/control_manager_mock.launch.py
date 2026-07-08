from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("k1muse_control_manager"),
        "config",
        "control_manager.mock.yaml",
    ])

    return LaunchDescription([
        Node(
            package="k1muse_control_manager",
            executable="control_manager_node",
            name="control_manager_node",
            output="screen",
            parameters=[config],
        ),
    ])
