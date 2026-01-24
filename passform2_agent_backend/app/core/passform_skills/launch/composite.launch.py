import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
import launch_ros.event_handlers

import lifecycle_msgs.msg

def generate_launch_description():
    ld = LaunchDescription()

    # Name des neuen Rust-Pakets
    package_name = 'passform_skills_rs'

    # Pfade zu den Ressourcen innerhalb des Rust-Paket-Shares
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'params.yaml'
    )
    
    skill_file_path = os.path.join(
        get_package_share_directory(package_name),
        'skills',
        'pick_and_place.yaml'
    )

    # Der Rust Lifecycle Node
    # Wichtig: 'executable' muss dem Namen in deiner Cargo.toml entsprechen
    action_node = launch_ros.actions.LifecycleNode(
        package=package_name,
        name="example_composite",
        namespace="",
        executable="example_composite", 
        parameters=[
            {'skill_file': skill_file_path},
            config
        ],
        output='screen',
        emulate_tty=True # Sorgt für farbiges Logging in Rust
    )

    # Event Handler: Wenn der Node 'inactive' erreicht (nach configure), direkt 'activate' aufrufen
    register_event_handler_for_activation = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=action_node, 
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="Rust Node 'inactive' -> Sende 'activate' Transition..."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(action_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Event Handler: Bestätigung wenn 'active' erreicht wurde
    register_event_handler_for_active_logging = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=action_node, 
            goal_state='active',
            entities=[
                launch.actions.LogInfo(msg="Rust Node ist jetzt 'active' und bereit für Goals."),
            ],
        )
    )

    # Initiales Event: Node in den 'configure' Status versetzen
    event_to_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(action_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Aktionen zur Description hinzufügen
    ld.add_action(register_event_handler_for_activation)
    ld.add_action(register_event_handler_for_active_logging)
    ld.add_action(action_node)
    ld.add_action(event_to_configure)

    return ld