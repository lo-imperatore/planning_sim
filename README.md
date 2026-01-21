# planning_sim

Containerized ROS 2 and Gazebo simulation workspace.

## Requirements

- Docker
- Docker Compose
- Linux system with X11

## Build the Container

```bash
docker compose build
```

## Start the Container

```bash
docker compose up -d
```

## Access the Container

```bash
docker exec -it ros-gz-sim bash
```

## Build the Workspace

```bash
cd /work/ws
colcon build
source install/setup.bash
```

## Run the Simulation

```bash
ros2 launch <package_name> <launch_file>.launch.py
```

Example:

```bash
ros2 launch planning_sim sim.launch.py
```

## Stop the Container

```bash
docker compose down
```
