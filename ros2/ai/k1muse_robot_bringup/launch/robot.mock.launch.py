from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node, SetParameter
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    autostart = LaunchConfiguration('autostart')

    # Audio IO node (mock device) -- LifecycleNode
    audio_io = LifecycleNode(
        package='k1muse_voice_audio',
        executable='audio_io_node',
        name='audio_io',
        namespace='',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('k1muse_voice_audio'),
                'config', 'audio.mock.yaml'
            ])
        ],
    )
    audio_io_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=audio_io,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(audio_io),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                    condition=IfCondition(autostart),
                )
            ],
        )
    )
    audio_io_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=audio_io,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(audio_io),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                    condition=IfCondition(autostart),
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),

        # Include ai_runtime mock launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('k1muse_ai_runtime'),
                    'launch', 'ai_runtime.mock.launch.py'
                ])
            ])
        ),

        # Audio IO node (mock device) -- LifecycleNode with auto configure+activate
        audio_io,
        audio_io_configure,
        audio_io_activate,

        # Intent node
        Node(
            package='k1muse_voice_intent',
            executable='intent_node',
            name='intent_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_voice_intent'),
                    'config', 'intent.mock.yaml'
                ])
            ],
        ),

        # Multimodal Supervisor
        Node(
            package='k1muse_multimodal_supervisor',
            executable='supervisor_node',
            name='multimodal_supervisor',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_multimodal_supervisor'),
                    'config', 'supervisor.mock.yaml'
                ])
            ],
        ),

        # Task Manager
        Node(
            package='k1muse_task_manager',
            executable='task_manager_node',
            name='task_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_task_manager'),
                    'config', 'task_manager.mock.yaml'
                ])
            ],
        ),

        # Control Manager
        Node(
            package='k1muse_control_manager',
            executable='control_manager_node',
            name='control_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_control_manager'),
                    'config', 'control_manager.mock.yaml'
                ])
            ],
        ),

        # Reminder Node
        Node(
            package='k1muse_voice_reminder',
            executable='reminder_node',
            name='reminder_node',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_voice_reminder'),
                    'config', 'reminder.mock.yaml'
                ])
            ],
        ),

        # Vision 3D Node
        Node(
            package='k1muse_vision_3d',
            executable='vision_3d_node',
            name='vision_3d',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_vision_3d'),
                    'config', 'vision_3d.mock.yaml'
                ])
            ],
        ),

        # Mock Audio Scenario
        Node(
            package='k1muse_mock_devices',
            executable='audio_scenario_node',
            name='audio_scenario',
            parameters=[{'scenario': 'wake_speech', 'publish_rate_ms': 20}],
        ),

        # Mock Camera Scenario
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

        # Mock Depth Camera Scenario
        Node(
            package='k1muse_mock_devices',
            executable='depth_camera_scenario_node',
            name='depth_camera_scenario',
            parameters=[{'frame_id': 'depth_camera_optical_frame'}],
        ),
    ])
