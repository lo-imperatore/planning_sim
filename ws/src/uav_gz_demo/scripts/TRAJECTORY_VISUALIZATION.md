# Trajectory Visualization in Gazebo

This system provides real-time visualization of drone trajectories in both Gazebo and RViz while the robot is following the planned path.

## Files

- `trajectory_visualizer.py` - ROS marker-based visualizer (works with RViz)
- `gazebo_trajectory_visualizer.py` - Direct Gazebo visualization using SDF models
- `test_trajectory_viz.py` - Create test trajectories for visualization testing

## Features

### ROS Marker Visualization (`trajectory_visualizer.py`)
- **Line Strip**: Shows the complete trajectory path as a continuous line
- **Waypoint Spheres**: Individual markers at each trajectory point
- **Orientation Arrows**: Shows drone heading at each waypoint
- **Start/End Markers**: Special markers for trajectory start (green) and end (red)
- **Configurable Colors**: Customizable colors for different elements
- **RViz Compatible**: Works seamlessly with RViz for debugging

### Gazebo Direct Visualization (`gazebo_trajectory_visualizer.py`)
- **Direct SDF Models**: Creates actual 3D objects in Gazebo scene
- **Cylindrical Path**: Shows trajectory as connected cylindrical segments
- **Waypoint Spheres**: 3D spheres at trajectory points
- **Persistent Display**: Remains visible throughout simulation
- **No Bridge Required**: Works directly with Gazebo services

## Configuration

### Launch File Integration

The trajectory visualizers are automatically included in your launch file with these parameters:

```python
# ROS Marker Visualizer
trajectory_visualizer = Node(
    package='uav_gz_demo',
    executable='trajectory_visualizer.py',
    parameters=[{
        'csv_path': traj_csv,              # Path to trajectory CSV
        'frame_id': 'world',               # Coordinate frame
        'marker_scale': 0.3,               # Size of markers
        'line_width': 0.15,                # Thickness of trajectory line
        'trajectory_color': [1.0, 0.2, 0.2, 0.8],  # RGBA color
        'show_orientation': True,          # Show heading arrows
        'show_waypoints': True,            # Show waypoint spheres
    }]
)

# Gazebo Direct Visualizer
gazebo_trajectory_visualizer = Node(
    package='uav_gz_demo',
    executable='gazebo_trajectory_visualizer.py',
    parameters=[{
        'csv_path': traj_csv,              # Path to trajectory CSV
        'world_name': 'default',           # Gazebo world name
        'line_color': [1.0, 0.0, 0.0, 0.8],       # Line color
        'waypoint_color': [0.0, 1.0, 0.0, 0.8],   # Waypoint color
        'sphere_radius': 0.15,             # Waypoint sphere size
    }]
)
```

### Trajectory CSV Format

The trajectory file should have this format:
```csv
time_s,x,y,z,yaw_rad,roll_rad,pitch_rad
0.0,-9.5,12.0,1.5,0.0,0.0,0.0
0.1,-9.4,11.8,1.5,0.1,0.0,0.0
...
```

## Usage

### Basic Usage

1. **Launch with your existing trajectory**:
   ```bash
   ros2 launch uav_gz_demo x3_with_traj_launch_gp.py traj_csv:=/path/to/your/trajectory.csv
   ```

2. **The trajectory will be automatically visualized in**:
   - Gazebo GUI (red line with green waypoints)
   - RViz (if you open it and add MarkerArray display for `/trajectory_markers`)

### Creating Test Trajectories

Create test trajectories for visualization:

```bash
# Circular trajectory
python3 /home/pietro/planning_sim/ws/src/uav_gz_demo/scripts/test_trajectory_viz.py circle

# Spiral trajectory
python3 /home/pietro/planning_sim/ws/src/uav_gz_demo/scripts/test_trajectory_viz.py spiral

# Warehouse navigation trajectory
python3 /home/pietro/planning_sim/ws/src/uav_gz_demo/scripts/test_trajectory_viz.py warehouse
```

Then launch with the test trajectory:
```bash
ros2 launch uav_gz_demo x3_with_traj_launch_gp.py traj_csv:=/home/pietro/planning_sim/ws/src/uav_gz_demo/trajectories/test_warehouse.csv
```

### RViz Visualization

To see markers in RViz:

1. **Launch RViz**:
   ```bash
   rviz2
   ```

2. **Add MarkerArray display**:
   - Click "Add" → "By topic" → "/trajectory_markers" → "MarkerArray"

3. **Set Fixed Frame**: `world`

4. **You should see**:
   - Red trajectory line
   - Red waypoint spheres
   - Blue orientation arrows
   - Green start marker
   - Red end marker

## Customization

### Colors

Modify colors in the launch file parameters:
- `trajectory_color`: [R, G, B, A] values (0.0-1.0)
- `waypoint_color`: Color for waypoint spheres
- `line_color`: Color for Gazebo trajectory line

### Sizes

- `marker_scale`: Overall size multiplier for markers
- `line_width`: Thickness of trajectory line
- `sphere_radius`: Size of waypoint spheres

### Visibility Options

- `show_orientation`: Enable/disable heading arrows
- `show_waypoints`: Enable/disable waypoint spheres
- `publish_rate`: How often to update markers (Hz)

## Troubleshooting

### No Trajectory Visible in Gazebo

1. **Check if visualizer started**:
   ```bash
   ros2 node list | grep trajectory
   ```

2. **Check Gazebo services**:
   ```bash
   gz service -l | grep create
   ```

3. **Manual trajectory spawn**:
   ```bash
   ros2 run uav_gz_demo gazebo_trajectory_visualizer.py --ros-args -p csv_path:=/path/to/trajectory.csv
   ```

### No Markers in RViz

1. **Check topic**:
   ```bash
   ros2 topic echo /trajectory_markers
   ```

2. **Check frame ID**: Ensure RViz fixed frame matches trajectory frame

3. **Check marker bridge**: Verify `/trajectory_markers` bridge is working

### Trajectory Not Loading

1. **Check CSV file path**:
   ```bash
   ls -la /path/to/your/trajectory.csv
   ```

2. **Check CSV format**: Ensure file has correct columns

3. **Check logs**:
   ```bash
   ros2 node info /trajectory_visualizer
   ```

## Integration with Your Trajectory Planner

The visualization system automatically loads the same CSV file your global planner uses, so:

1. **Your planner publishes commands** → Robot follows trajectory
2. **Visualizer reads same CSV** → Shows planned path
3. **Both systems synchronized** → See plan vs. execution

This gives you real-time feedback on how well your drone is following the planned trajectory.

## Advanced Features

### Dynamic Trajectory Updates

If your trajectory changes during flight, restart the visualizer:
```bash
ros2 lifecycle set /trajectory_visualizer shutdown
ros2 run uav_gz_demo trajectory_visualizer.py --ros-args -p csv_path:=/new/trajectory.csv
```

### Multiple Trajectories

Run multiple visualizers with different namespaces:
```bash
ros2 run uav_gz_demo trajectory_visualizer.py --ros-args -r __ns:=/drone1 -p csv_path:=/drone1_traj.csv
ros2 run uav_gz_demo trajectory_visualizer.py --ros-args -r __ns:=/drone2 -p csv_path:=/drone2_traj.csv
```

### Performance Tuning

For very long trajectories, reduce update rate:
- Set `publish_rate: 0.5` for 2-second updates
- Set `show_waypoints: false` to reduce marker count
- Use every Nth waypoint for large datasets