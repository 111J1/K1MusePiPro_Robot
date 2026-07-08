from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    audio_io_node = LifecycleNode(
        package='k1muse_voice_audio',
        executable='audio_io_node',
        name='audio_io',
        namespace='',
        parameters=[PathJoinSubstitution([
            FindPackageShare('k1muse_voice_audio'), 'config', 'audio.mock.yaml'
        ])],
        autostart=False,
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('k1muse_ai_runtime'),
                    'launch', 'ai_runtime.mock.launch.py'
                ])
            ])
        ),
        audio_io_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=audio_io_node,
                on_exit=[
                    LogInfo(msg='[audio_io] exited'),
                ],
            )
        ),
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(audio_io_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=audio_io_node,
                on_exit=[
                    LogInfo(msg='[audio_io] process exited'),
                ],
            )
        ),
        Node(
            package='k1muse_voice_intent',
            executable='intent_node',
            name='intent_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('k1muse_voice_intent'), 'config', 'intent.mock.yaml'
            ])],
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
            package='k1muse_mock_devices',
            executable='audio_scenario_node',
            name='audio_scenario',
            parameters=[{'scenario': 'wake_speech'}],
        ),
    ])
