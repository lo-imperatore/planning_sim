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
        default_value=os.path.join(pkg_share, 'worlds', 'x3_warehouse_challenging.sdf'),
        description='Path to the world SDF'
    )
    traj_arg = DeclareLaunchArgument(
        'traj_csv',
        default_value=os.path.join(pkg_share, 'trajectories', 'traj_drone_hybrid_2.csv'),
        description='Trajectory CSV (time_s,x,y,z,yaw_rad,roll_rad,pitch_rad)'
    )
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='50.0')
    print_every_arg = DeclareLaunchArgument('print_every_s', default_value='0.5')
    use_steady_arg = DeclareLaunchArgument('use_steady_clock', default_value='true')

    # GUI config to auto-load MarkerManager (so /marker exists)
    gui_cfg_default = os.path.join(pkg_share, 'config', 'gui_markers.config')
    gui_cfg_arg = DeclareLaunchArgument(
        'gui_config', default_value=gui_cfg_default,
        description='GUI config that loads the Markers plugin'
    )

    # ---- robot spawn arguments ----
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='-10.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='12.0')
    z_spawn_arg = DeclareLaunchArgument('z_spawn', default_value='1.2')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')

    world = LaunchConfiguration('world')
    traj_csv = LaunchConfiguration('traj_csv')
    rate_hz = LaunchConfiguration('rate_hz')
    gui_config = LaunchConfiguration('gui_config')

    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    z_spawn = LaunchConfiguration('z_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')

    # Sim time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

    # ---- Gazebo world only (without robot) ----
    # forward --gui-config so the Markers GUI loads
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'world': world,
                          'gz_args': [world, ' --gui-config ', gui_config]}.items()
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ---- ROS ↔ GZ bridges (unchanged) ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/default/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/X3/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/X3/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    enable_once = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-1', '/X3/enable', 'std_msgs/Bool', '{data: true}'],
        output='screen'
    )

    # ---- global planner (publishes Twist) ----
    global_planner = Node(
        package='uav_gz_demo',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[{
            'ns': 'X3',
            'csv_path': traj_csv,
            'twist_topic': '/X3/gazebo/command/twist',
            'enable_topic': '/X3/enable',
            'rate_hz': rate_hz,
            'use_sim_time': use_sim_time,
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
            'use_sim_time': use_sim_time,
        }]
    )

    # start the marker sender a little after Gazebo is up
    traj_marker_delayed = TimerAction(period=2.0, actions=[traj_to_gz_markers])

    return LaunchDescription([
        world_arg, traj_arg, rate_arg, print_every_arg, use_steady_arg,
        gui_cfg_arg,
        gz,
        x_spawn_arg, y_spawn_arg, z_spawn_arg, yaw_spawn_arg, is_use_sim_time_arg,
        spawn_entity,
        bridge,
        enable_once,
        TimerAction(period=2.3, actions=[global_planner]),
        # trajectory_visualizer,
        traj_marker_delayed,
    ])
