from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('k1muse_ai_runtime'),
                    'launch', 'ai_runtime.mock.launch.py'
                ])
            ])
        ),
        Node(
            package='k1muse_multimodal_supervisor',
            executable='supervisor_node',
            name='multimodal_supervisor',
            parameters=[PathJoinSubstitution([
                FindPackageShare('k1muse_multimodal_supervisor'), 'config', 'supervisor.mock.yaml'
            ])],
        ),
        Node(
            package='k1muse_vision_3d',
            executable='vision_3d_node',
            name='vision_3d',
            parameters=[PathJoinSubstitution([
                FindPackageShare('k1muse_vision_3d'), 'config', 'vision_3d.mock.yaml'
            ])],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mock_depth_camera_tf',
            arguments=[
                '--x', '0.140406',
                '--y', '0.0',
                '--z', '0.05',
                '--roll', '-1.57079632679',
                '--pitch', '0.0',
                '--yaw', '-1.57079632679',
                '--frame-id', 'base_link',
                '--child-frame-id', 'depth_camera_optical_frame',
            ],
        ),
        Node(
            package='k1muse_mock_devices',
            executable='camera_scenario_node',
            name='camera_scenario',
            parameters=[{'frame_id': 'depth_camera_optical_frame'}],
        ),
        Node(
            package='k1muse_mock_devices',
            executable='depth_camera_scenario_node',
            name='depth_camera_scenario',
            parameters=[{'frame_id': 'depth_camera_optical_frame'}],
        ),
    ])
