# launch/x3_with_pinger.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_gz_demo')
    world = os.path.join(pkg, 'worlds', 'x3_warehouse.sdf')

    # Gazebo (server+GUI) using your gz_sim.launch.py
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Bridges (Twist + enable + clock + pose)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
        ],
        output='screen',
    )

    # One-shot enable after the bridge is up
    enable_once = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/X3/enable', 'std_msgs/Bool', '{data: true}'],
        output='screen'
    )

    # Your pinger node (publishes Twist). Two options:
    # 1) If your node supports a "topic" param (recommended), set it here:
    pinger = Node(
        package='uav_gz_demo',
        executable='cmdvel_pinger',
        name='cmdvel_pinger',
        output='screen',
        parameters=[{'topic': '/X3/gazebo/command/twist', 'rate': 10.0}],  # 'rate' is optional if your node accepts it
    )

    position_ctl = Node(
        package='uav_gz_demo',
        executable='x3_position_controller',
        name='x3_position_controller',
        output='screen',
        parameters=[{
            'ns': 'X3',
            'world_frame': 'world',
            'body_frame': 'X3/base_link',
            'kp_xy': 1.0, 'kp_z': 1.0, 'kp_yaw': 1.5,
            'max_v_xy': 1.0, 'max_v_z': 0.8, 'max_w_z': 1.0,
            'pos_tol': 0.10, 'yaw_tol': 0.08,
            'rate': 50.0,
            'use_sim_time': True
        }]
    )


    return LaunchDescription([
        gz,
        bridge,
        # small delays to avoid races: arm, then start the pinger
        TimerAction(period=1.5, actions=[enable_once]),
        TimerAction(period=2.0, actions=[pinger]),
        # TimerAction(period=1.8, actions=[position_ctl]),
    ])
