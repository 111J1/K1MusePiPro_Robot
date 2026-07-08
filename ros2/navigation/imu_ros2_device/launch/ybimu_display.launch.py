from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_path('imu_ros2_device')
    default_rviz_config_path = package_path / 'rviz/ybimu.rviz'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    device_node = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        parameters=[{
            'port': '/dev/myimu',
            'frame_id': 'imu_link',
            'publish_rate': 10.0,
        }],
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
        device_node,
    ])
