        import os
        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
        from launch_ros.substitutions import FindPackageShare
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        
        def generate_launch_description():
            gui_arg = DeclareLaunchArgument(
                name='gui',
                default_value='true',
                description='Show Gazebo GUI'
            )
        
            pkg_share = FindPackageShare('iiwa_description')
        
            world_path = PathJoinSubstitution([
                pkg_share,
                'gazebo',
                'worlds',
                'empty.world'
            ])
        
            gz_sim_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ros_gz_sim'),
                        'launch',
                        'gz_sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'gui': LaunchConfiguration('gui'),
                    'gz_args': [TextSubstitution(text='-r '), world_path],
                }.items(),
            )
        
            return LaunchDescription([
                gui_arg,
                gz_sim_launch
            ])
