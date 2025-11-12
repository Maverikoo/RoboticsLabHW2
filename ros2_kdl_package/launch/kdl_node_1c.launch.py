from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ctrl = DeclareLaunchArgument(
        name='ctrl',
        default_value='velocity_ctrl',
        description='Controller to use: velocity_ctrl or velocity_ctrl_null'
    )

    cmd_interface = DeclareLaunchArgument(
        name='cmd_interface',
        default_value='velocity',  # position | velocity | effort
        description='Command interface of the robot controllers'
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("ros2_kdl_package"),
            "config",
            "kdl_param.yaml",
        ]),
        description="YAML"
    )

    ros2_kdl_node_1c = Node(
        package="ros2_kdl_package",
        executable="ros2_kdl_node_1c",
        name="ros2_kdl_node_1c",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"ctrl": LaunchConfiguration("ctrl")},
            {"cmd_interface": LaunchConfiguration("cmd_interface")},
        ],
    )

    return LaunchDescription([
        ctrl,
        cmd_interface,
        params_file_arg,
        ros2_kdl_node_1c,
    ])