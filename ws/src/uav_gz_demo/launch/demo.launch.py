from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_gz_demo')
    world = os.path.join(pkg, 'worlds', 'x3_warehouse.sdf')

    # Gazebo (server + GUI)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('uav_gz_demo'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Bridges:
    #  - /clock: Gazebo -> ROS
    #  - /world/default/pose/info: Gazebo -> ROS (TFMessage)
    #  - /model/quadrotor/cmd_vel: ROS -> Gazebo
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

    node = Node(
        package='uav_gz_demo',
        executable='cmdvel_pinger',
        output='screen'
    )

    return LaunchDescription([gz, bridge, node])
