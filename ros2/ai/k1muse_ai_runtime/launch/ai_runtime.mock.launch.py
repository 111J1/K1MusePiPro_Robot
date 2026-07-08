from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration("autostart")
    config = PathJoinSubstitution(
        [FindPackageShare("k1muse_ai_runtime"), "config", "ai_runtime.mock.yaml"]
    )
    runtime = LifecycleNode(
        package="k1muse_ai_runtime",
        executable="ai_runtime_node",
        name="ai_runtime",
        namespace="",
        output="screen",
        parameters=[config],
    )
    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=runtime,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(runtime),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                    condition=IfCondition(autostart),
                )
            ],
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=runtime,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(runtime),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                    condition=IfCondition(autostart),
                )
            ],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="true"),
            runtime,
            configure,
            activate,
        ]
    )
