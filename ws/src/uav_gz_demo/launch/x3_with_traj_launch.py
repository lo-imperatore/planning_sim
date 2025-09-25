from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnExecutionComplete, OnProcessExit
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
        default_value=os.path.join(pkg_share, 'worlds', 'x3_warehouse_challenging.sdf'),
        description='Path to the world SDF'
    )
    traj_arg = DeclareLaunchArgument(
        'traj_csv',
        default_value=os.path.join(pkg_share, 'trajectories', 'traj_drone_hybrid_3.csv'),
        description='Trajectory CSV (x,y,z,roll_rad,pitch_rad,yaw_rad)'
    )
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='50.0')
    print_every_arg = DeclareLaunchArgument('print_every_s', default_value='0.5')
    use_steady_arg = DeclareLaunchArgument('use_steady_clock', default_value='true')
    setpoint_topic_arg = DeclareLaunchArgument('setpoint_topic', default_value='/X3/position_setpoint')
    
    # ---- robot spawn arguments ----
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='-10.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='12.0')
    z_spawn_arg = DeclareLaunchArgument('z_spawn', default_value='1.2')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='1.57')

    world = LaunchConfiguration('world')
    traj_csv = LaunchConfiguration('traj_csv')
    rate_hz = LaunchConfiguration('rate_hz')
    print_every_s = LaunchConfiguration('print_every_s')
    use_steady_clock = LaunchConfiguration('use_steady_clock')
    setpoint_topic = LaunchConfiguration('setpoint_topic')
    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    z_spawn = LaunchConfiguration('z_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')

    # ---- Gazebo world only (without robot) ----
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'world': world}.items()
    )
    
    # ---- Spawn X3 drone separately ----
    x3_model_path = os.path.join(pkg_share, 'models', 'X3', 'model.sdf')
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'X3',
            '-file', x3_model_path,
            '-x', x_spawn,
            '-y', y_spawn,
            '-z', z_spawn,
            '-Y', yaw_spawn
        ],
        output='screen'
    )

    # ---- ROS ↔ GZ bridges ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # Clock (Gazebo → ROS)
            # '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/X3/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose',
            # '/model/X3/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
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
            'world_frame': 'default',
            'body_frame': 'X3',
            'kp_xy': 1.0, 'kp_z': 1.0, 'kp_yaw': 1.5,
            'max_v_xy': 1.0, 'max_v_z': 0.8, 'max_w_z': 1.0,
            'pos_tol': 0.10, 'yaw_tol': 0.08,
            'rate': rate_hz,
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
            'rate_hz': rate_hz,
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

    traj_to_gz_markers = Node(
        package='uav_gz_demo',
        executable='trajectory_visualizer',   # <— your converter node
        name='trajectory_visualizer',
        output='screen',
        parameters=[{
            'csv_path': traj_csv,
            'marker_service': '/marker',   # must match <topic_name> in gui_markers.config
            'line_width': 0.15,
            'point_size': 0.30,
            'traj_color': [1.0, 0.2, 0.2, 0.8],
            'show_waypoints': True,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        world_arg, traj_arg, rate_arg, print_every_arg, use_steady_arg, setpoint_topic_arg,
        gz, x_spawn_arg, y_spawn_arg, z_spawn_arg, yaw_spawn_arg,
        spawn_entity,
        bridge,
        enable_once,
        position_ctl,
        traj_to_gz_markers,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=1.1, 
                        actions=[traj_streamer]
                    ),
                ]
            )
        ),
        # TimerAction(period=1.5, actions=[enable_once]),
        # TimerAction(period=1.8, actions=[position_ctl]),
        # TimerAction(period=2.2, actions=[traj_streamer]),
    ])
