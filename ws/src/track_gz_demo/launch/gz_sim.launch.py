from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Args (no conditions)
    verbose  = LaunchConfiguration('verbose')
    world    = LaunchConfiguration('world')
    extra    = LaunchConfiguration('extra_resource_paths')  # optional colon-separated

    pkg_share = FindPackageShare('track_gz_demo')
    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'csiro_in_the_forest.sdf'])

    # Fuel cache (for ground_plane/sun, harmless if unused)
    fuel_cache = os.path.join(os.path.expanduser('~'), '.gz', 'fuel')

    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='4', description='Gazebo verbosity 0..4'),
        DeclareLaunchArgument('world',   default_value=default_world, description='Path to world .sdf'),
        DeclareLaunchArgument('extra_resource_paths', default_value='',
                              description='Optional colon-separated extra resource paths'),

        # Compose GZ_SIM_RESOURCE_PATH without PythonExpression/IfCondition
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                pkg_share, ':',
                PathJoinSubstitution([pkg_share, 'worlds']), ':',
                PathJoinSubstitution([pkg_share, 'models']), ':',
                fuel_cache, ':',
                extra
            ]
        ),

        # Single process: always GUI (no IfCondition)
        ExecuteProcess(
            cmd=['gz', 'sim', world, '-v', verbose, '-r'],
            output='screen'
        ),
    ])
