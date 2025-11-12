import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Trova il percorso del tuo file .yaml
    config_file = os.path.join(
        get_package_share_directory('ros2_kdl_package'),
        'config',
        'client_goal.yaml'
    )

    # 2. Definisci il nodo del client
    client_node = Node(
        package='ros2_kdl_package',
        executable='execute_trajectory_client',
        name='execute_trajectory_client', # Corrisponde al nome nel .yaml
        output='screen',
        parameters=[config_file]  # <-- QUESTA È LA MAGIA!
    )

    # 3. Restituisci la descrizione per avviare tutto
    return LaunchDescription([
        client_node
    ])