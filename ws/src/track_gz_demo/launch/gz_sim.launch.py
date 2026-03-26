from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    verbose = LaunchConfiguration('verbose')
    world = LaunchConfiguration('world')
    extra = LaunchConfiguration('extra_resource_paths')

    pkg_share = FindPackageShare('track_gz_demo')
    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'desert_world.sdf'])
    bridge_yaml = PathJoinSubstitution([pkg_share, 'config', 'gz_bridge.yaml'])

    fuel_cache = os.path.join(os.path.expanduser('~'), '.gz', 'fuel')

    return LaunchDescription([
        DeclareLaunchArgument(
            'verbose',
            default_value='4',
            description='Gazebo verbosity 0..4'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to world .sdf'
        ),
        DeclareLaunchArgument(
            'extra_resource_paths',
            default_value='',
            description='Optional colon-separated extra resource paths'
        ),

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

        ExecuteProcess(
            cmd=['gz', 'sim', world, '-v', verbose, '-r'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[
                {'config_file': bridge_yaml},
                {'expand_gz_topic_names': False},
            ]
        ),
         Node(
            package='track_gz_demo',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            parameters=[
                {'odom_topic': '/odom'},
                {'parent_frame': 'odom'},
                {'child_frame': 'base_link'},
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_nav_imu',
            arguments=[
               '--x', '0.426909', '--y', '-0.005178', '--z', '0.423',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'CSIRO/nav_imu/camera_front'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_encoder_rotating',
            arguments=[
                '--x', '0.433', '--y', '0.0', '--z', '0.453797',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'encoder_rotating_link'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='encoder_to_gpu_lidar',
            arguments=[
                '--x', '0.008658', '--y', '0.0', '--z', '0.099161',
                '--roll', '-0.785398', '--pitch', '0.0', '--yaw', '-1.5707963267948966',
                '--frame-id', 'encoder_rotating_link',
                '--child-frame-id', 'CSIRO/encoder_rotating_link/gpu_lidar'
            ]
        )
    ])