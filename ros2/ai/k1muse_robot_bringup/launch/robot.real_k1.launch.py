import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
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
    no_motion = LaunchConfiguration('no_motion')
    autostart = LaunchConfiguration('autostart')

    # LD_PRELOAD for llama-server (notcm.so) -- only set if the file exists.
    # If missing, log a warning so all other processes still start normally.
    _notcm_path = '/home/bianbu/.local/lib/notcm.so'
    _ld_preload_actions = []
    if os.path.isfile(_notcm_path):
        _ld_preload_actions.append(
            SetEnvironmentVariable('LD_PRELOAD', _notcm_path)
        )
    else:
        _ld_preload_actions.append(
            LogInfo(msg=[
                'LD_PRELOAD: notcm.so not found at ', _notcm_path,
                ' -- skipping. Set LD_PRELOAD manually if llama-server needs it.'
            ])
        )

    # Audio IO node (ALSA) -- LifecycleNode
    audio_io = LifecycleNode(
        package='k1muse_voice_audio',
        executable='audio_io_node',
        name='audio_io',
        namespace='',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('k1muse_voice_audio'),
                'config', 'audio_io.real_k1.yaml'
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
        DeclareLaunchArgument('no_motion', default_value='true',
                              description='Disable real motion commands'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto configure+activate lifecycle nodes'),

        # SpacemiT EP environment variables
        SetEnvironmentVariable('SPACEMIT_EP_USE_GLOBAL_INTRA_THREAD', '1'),
        SetEnvironmentVariable('SPACEMIT_EP_INTRA_THREAD_NUM', '2'),
        SetEnvironmentVariable('SPACEMIT_EP_INTER_THREAD_NUM', '1'),

        # SDK library path (for libportaudio.so etc.)
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            '/home/bianbu/AI_SDK/ai-sdk/output/staging/lib:' +
            os.environ.get('LD_LIBRARY_PATH', '')),

        # LD_PRELOAD (conditional)
        *_ld_preload_actions,

        # Include ai_runtime real launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('k1muse_ai_runtime'),
                    'launch', 'ai_runtime.real_k1.launch.py'
                ])
            ])
        ),

        # Audio IO node (ALSA) -- LifecycleNode with auto configure+activate
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
                    'config', 'intent.real_k1.yaml'
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
                    'config', 'supervisor.real_k1.yaml'
                ])
            ],
        ),

        # Task Manager (no real motion in no-motion mode)
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

        # Control Manager (mock/sink mode for no-motion)
        Node(
            package='k1muse_control_manager',
            executable='control_manager_node',
            name='control_manager',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('k1muse_control_manager'),
                    'config', 'control_manager.mock.yaml'
                ]),
                {'no_motion': True},
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

        # Camera driver note:
        # The camera package is external to this workspace. It must provide:
        #   /camera/main/color/image_raw
        #   /camera/main/depth_registered/image_raw
        #   /camera/main/color/camera_info
        # with header.frame_id=depth_camera_optical_frame, and TF must provide
        # base_link -> depth_camera_optical_frame through k1muse_description
        # plus the current lift_joint state.
        # If the external driver uses different names, add explicit remaps in
        # that camera launch or in this bringup layer before starting runtime.
        #
        # No mock device nodes in real mode -- real camera/audio devices used
        # No MCU bridge -- no-motion guard
    ])
