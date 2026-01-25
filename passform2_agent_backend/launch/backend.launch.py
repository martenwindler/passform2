import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Pfade zu den Ressourcen-Paketen auflösen
    try:
        res_share = get_package_share_directory('passform_agent_resources')
    except Exception:
        # Fallback falls noch nicht installiert (lokale Entwicklung)
        res_share = 'src/passform_agent_resources'

    # 2. Spezifische Config-Dateien definieren
    params_path = os.path.join(res_share, 'config', 'params.yaml')
    bay_path = os.path.join(res_share, 'config', 'bay.yaml')
    tech_data_path = os.path.join(res_share, 'config', 'config__technical_data.yaml')
    poi_path = os.path.join(res_share, 'config', 'poi.yaml')

    # Gemeinsame Parameter-Liste für alle Nodes
    common_params = [params_path, bay_path, tech_data_path, poi_path]

    # 3. Die Liste der Nodes erstellen
    # Jedes Rust-Binary aus deinem src/bin/ wird hier als Node definiert
    nodes = [
        # Haupt-Backend (aus src/main.rs)
        Node(
            package='passform_agent_backend',
            executable='passform_agent_backend',
            name='main_backend',
            output='screen',
            parameters=common_params
        ),
        # Inventory Manager
        Node(
            package='passform_agent_backend',
            executable='inventory_manager',
            name='inventory_manager',
            output='screen',
            parameters=common_params
        ),
        # Module Manager
        Node(
            package='passform_agent_backend',
            executable='module_manager',
            name='module_manager',
            output='screen',
            parameters=common_params
        ),
        # NFC Manager (Hardware-nahe)
        Node(
            package='passform_agent_backend',
            executable='nfc_manager',
            name='nfc_manager',
            output='screen',
            parameters=common_params
        ),
        # Socket IO Manager (Kommunikation zur Außenwelt)
        Node(
            package='passform_agent_backend',
            executable='socket_io_manager',
            name='socket_io_manager',
            output='screen',
            parameters=common_params
        ),
        # Task Manager (Skill-Abfolge)
        Node(
            package='passform_agent_backend',
            executable='task_manager',
            name='task_manager',
            output='screen',
            parameters=common_params
        )
    ]

    return LaunchDescription(nodes)