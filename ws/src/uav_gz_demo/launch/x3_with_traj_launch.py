from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('uav_gz_demo')

    # ---- args you can override on the command line ----
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'x3_warehouse.sdf'),
        description='Path to the world SDF'
    )
    traj_arg = DeclareLaunchArgument(
        'traj_csv',
        default_value=os.path.join(pkg_share, 'trajectories', 'drone_trajectory_rpy_rad.csv'),
        description='Trajectory CSV (time_s,x,y,z,yaw_rad,roll_rad,pitch_rad)'
    )
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='50.0')
    print_every_arg = DeclareLaunchArgument('print_every_s', default_value='0.5')
    use_steady_arg = DeclareLaunchArgument('use_steady_clock', default_value='true')
    setpoint_topic_arg = DeclareLaunchArgument('setpoint_topic', default_value='/X3/position_setpoint')

    world = LaunchConfiguration('world')
    traj_csv = LaunchConfiguration('traj_csv')
    rate_hz = LaunchConfiguration('rate_hz')
    print_every_s = LaunchConfiguration('print_every_s')
    use_steady_clock = LaunchConfiguration('use_steady_clock')
    setpoint_topic = LaunchConfiguration('setpoint_topic')

    # ---- Gazebo (your existing entrypoint) ----
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # ---- ROS ↔ GZ bridges ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Clock (Gazebo → ROS)
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            # UAV state (ROS ↔ Gazebo)
            '/world/default/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # one-shot enable after bridge is up
    enable_once = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/X3/enable', 'std_msgs/Bool', '{data: true}'],
        output='screen'
    )

    # ---- position controller (subscribes to setpoints, publishes Twist) ----
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
            'use_sim_time': True,
            # output topic the controller publishes (must match the bridge):
            'twist_topic': '/X3/gazebo/command/twist',
            # input topic must match the streamer below:
            # 'setpoint_topic': setpoint_topic,
        }]
    )

    # ---- trajectory streamer (now with prints + steady clock) ----
    traj_streamer = Node(
        package='uav_gz_demo',
        executable='trajectory_streamer',
        name='trajectory_streamer',
        output='screen',
        parameters=[{
            'csv_path': traj_csv,
            'topic_name': setpoint_topic,
            'frame_id': 'world',
            'rate_hz': 5.0,
            'takeoff_prepend': False,
            'takeoff_height': 2.0,
            'takeoff_duration_s': 3.0,
            'takeoff_start_z0': 0.0,
            'force_zero_roll_pitch': True,
            'use_steady_clock': use_steady_clock,   # <<< new
            'print_every_s': print_every_s,         # <<< new
            'min_subscribers': 1,                   # <<< new
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        world_arg, traj_arg, rate_arg, print_every_arg, use_steady_arg, setpoint_topic_arg,
        gz,
        bridge,
        enable_once,
        position_ctl,
        traj_streamer,
        # TimerAction(period=1.5, actions=[enable_once]),
        # TimerAction(period=1.8, actions=[position_ctl]),
        # TimerAction(period=2.2, actions=[traj_streamer]),
    ])
