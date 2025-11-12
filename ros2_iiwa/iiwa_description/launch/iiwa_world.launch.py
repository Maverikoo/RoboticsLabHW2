from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- 1. CONFIGURAZIONE ---
    pkg_iiwa_description = FindPackageShare('iiwa_description')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    world_path = PathJoinSubstitution([pkg_iiwa_description, 'gazebo', 'worlds', 'empty.world'])
    xacro_file = PathJoinSubstitution([pkg_iiwa_description, "gazebo", "iiwa_gazebo.xacro"])

    # --- 2. ARGOMENTI ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_name = LaunchConfiguration('model_name')
    
    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('model_name', default_value='iiwa'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    # --- 3. ROBOT DESCRIPTION ---
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # --- 4. SIMULAZIONE E SPAWN ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])]),
        launch_arguments={'gui': LaunchConfiguration('gui'), 'gz_args': [TextSubstitution(text='-r '), world_path]}.items(),
    )

    node_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='both', parameters=[robot_description, {'use_sim_time': use_sim_time}],
    ) 

    node_spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', model_name, '-z', '-0.3', '-allow_renaming', 'true'],
    )

    node_jsb = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    node_vc = Node(package="controller_manager", executable="spawner", arguments=["velocity_controller", "-c", "/controller_manager"])

    # --- 5. BRIDGES (LA PARTE CRITICA CHE ABBIAMO SISTEMATO) ---

    # A. IMAGE BRIDGE (Gestisce l'immagine /camera_ee)
    # Usa 'ros_gz_image' perché gestisce meglio la conversione dei pixel
    node_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_image',
        arguments=['/camera_ee'],  # Prende '/camera_ee' da Gazebo e lo porta su ROS
        output='screen'
    )

    # B. PARAMETER BRIDGE (Gestisce CameraInfo e Servizi)
    # Qui facciamo il remapping: Gazebo pubblica su "/camera_info", noi lo portiamo su ROS come "/camera_ee/camera_info"
    # così stanno nello stesso "namespace" dell'immagine.
    node_bridge_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_info',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V' # Opzionale: ground truth
        ],
        remappings=[
            ('/camera_info', '/camera_ee/camera_info') # <--- REMAPPING FONDAMENTALE
        ],
        output='screen'
    )

    # --- 6. ARUCO NODE ---
    node_aruco = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'marker_size': 0.20,
            'marker_id': 201,
            'marker_frame': 'aruco_marker_frame',
            'reference_frame': 'world',
            'camera_frame': 'camera_ee_link', # Assicurati che questo sia il nome nel tuo URDF
            'image_is_rectified': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # ArUco legge:
            ('/image', '/camera_ee/image_raw'),
            ('/camera_info', '/camera_ee/camera_info'), # Ora combacia col remapping del bridge
            
        ],
        output='screen'
    )

    # --- 7. EVENT HANDLERS ---
    delay_jsb = RegisterEventHandler(event_handler=OnProcessExit(target_action=node_spawn, on_exit=[node_jsb]))
    delay_vc = RegisterEventHandler(event_handler=OnProcessExit(target_action=node_jsb, on_exit=[node_vc]))

    return LaunchDescription(args + [
        gz_sim, node_rsp, node_spawn, 
        node_bridge_image, node_bridge_info, 
        node_aruco, delay_jsb, delay_vc
    ])