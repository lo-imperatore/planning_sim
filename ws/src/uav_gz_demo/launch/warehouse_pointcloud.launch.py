from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start RViz'
    )

    # Warehouse pointcloud publisher node
    warehouse_pointcloud_publisher = Node(
        package='uav_gz_demo',
        executable='warehouse_pointcloud_ros.py',
        name='warehouse_pointcloud_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'point_density': 0.15,
            'publish_rate': 1.0,
        }],
        remappings=[
            ('/warehouse/obstacles', '/planning/obstacles'),
            ('/warehouse/environment', '/planning/environment'),
        ]
    )

    # Static transform publisher node
    world_to_map_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # RViz node
    rviz = Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        arguments=['-d', '$(find uav_gz_demo)/config/warehouse_pointcloud.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        warehouse_pointcloud_publisher,
        world_to_map_transform,
        rviz
    ])