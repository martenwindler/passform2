import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch.events  # noqa: E402

import launch_ros.actions  # noqa: E402
import launch_ros.events  # noqa: E402
import launch_ros.events.lifecycle  # noqa: E402

import lifecycle_msgs.msg

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('passform_skills'),
        'config',
        'params.yaml'
        )

    action_node = launch_ros.actions.LifecycleNode(
        package="passform_skills",
        name="example_primitive",
        namespace="",
        executable="example_primitive",
        parameters=[
                {'skill_file': os.path.join(get_package_share_directory('passform_skills'),'skills','pick.yaml')},
                config
            ],
        # arguments=['--ros-args', '--log-level', 'debug']

    )

    # When the node reaches the 'active' state, log a message and deactivate.
    register_event_handler_for_talker_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=action_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node reached the 'active' state, shutting it down again."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(action_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
                )),
            ],
        )
    )

    # When node reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_talker_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=action_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(action_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the node take the 'configure' transition.
    event_to_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(action_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    ld.add_action(register_event_handler_for_talker_reaches_inactive_state)
    # ld.add_action(register_event_handler_for_talker_reaches_active_state) # adding this will cause infinity loop
    ld.add_action(action_node)
    ld.add_action(event_to_configure)

    return ld
