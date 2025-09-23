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
        default_value=os.path.join(pkg_share, 'trajectories', 'traj_drone_hybrid_prova.csv'),
        description='Trajectory CSV (time_s,x,y,z,yaw_rad,roll_rad,pitch_rad)'
    )
    rate_arg = DeclareLaunchArgument('rate_hz', default_value='10.0')
    print_every_arg = DeclareLaunchArgument('print_every_s', default_value='0.5')
    use_steady_arg = DeclareLaunchArgument('use_steady_clock', default_value='true')

    
    # ---- robot spawn arguments ----
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='3.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='0.0')
    z_spawn_arg = DeclareLaunchArgument('z_spawn', default_value='1.2')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')

    world = LaunchConfiguration('world')
    traj_csv = LaunchConfiguration('traj_csv')
    rate_hz = LaunchConfiguration('rate_hz')
    
    x_spawn = LaunchConfiguration('x_spawn')
    y_spawn = LaunchConfiguration('y_spawn')
    z_spawn = LaunchConfiguration('z_spawn')
    yaw_spawn = LaunchConfiguration('yaw_spawn')
    
    # Sim time parameter
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')

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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # one-shot enable after bridge is up
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
            # output topic the controller publishes (must match the bridge):
            
            # input topic must match the streamer below:
            # 'setpoint_topic': setpoint_topic,
        }]
    )

    return LaunchDescription([
        world_arg, traj_arg, rate_arg, print_every_arg, use_steady_arg,
        gz, x_spawn_arg, y_spawn_arg, z_spawn_arg, yaw_spawn_arg, is_use_sim_time_arg,
        spawn_entity,
        bridge,
        TimerAction(period=1.5, actions=[enable_once]),
        TimerAction(period=3.0, actions=[global_planner])

    ])
