# SDF to Pointcloud Generator

This directory contains Python scripts to generate pointcloud representations from SDF (Simulation Description Format) files, specifically designed for the warehouse environment.

## Files

- `sdf_to_pointcloud.py` - Main library for parsing SDF files and generating pointclouds
- `warehouse_pointcloud.py` - Simple script specifically for the warehouse SDF
- `requirements.txt` - Python dependencies
- `README.md` - This file

## Installation

Install the required Python packages:

```bash
pip install -r requirements.txt
```

## Usage

### Quick Start - Warehouse Pointcloud

Generate a pointcloud from the warehouse SDF file:

```bash
cd /home/pietro/planning_sim/ws/src/uav_gz_demo/scripts
python3 warehouse_pointcloud.py
```

This will:
- Parse the warehouse SDF file
- Generate a pointcloud with 15cm point spacing
- Save the pointcloud in both TXT and PLY formats
- Create a 3D visualization
- Save files to `../pointclouds/` directory

### Advanced Options

Generate a dense pointcloud (5cm spacing) for detailed path planning:

```bash
python3 warehouse_pointcloud.py dense
```

Analyze the navigation space:

```bash
python3 warehouse_pointcloud.py analyze
```

### General SDF to Pointcloud

Use the general script for any SDF file:

```bash
python3 sdf_to_pointcloud.py <sdf_file> [options]
```

Options:
- `--density 0.1` - Set point density (distance between points in meters)
- `--output filename` - Set output filename (without extension)
- `--format txt|ply|both` - Choose output format
- `--visualize` - Display 3D visualization
- `--point-size 2.0` - Set point size for visualization

Example:
```bash
python3 sdf_to_pointcloud.py ../worlds/x3_warehouse_challenging.sdf --density 0.1 --visualize --format both
```

## Output Formats

### TXT Format
Simple ASCII format with X, Y, Z coordinates:
```
X Y Z
-20.000000 -15.000000 0.000000
-20.000000 -15.000000 0.100000
...
```

### PLY Format
Standard PLY (Polygon File Format) for 3D data:
```
ply
format ascii 1.0
element vertex 15234
property float x
property float y
property float z
end_header
-20.000000 -15.000000 0.000000
...
```

## Supported Geometries

The scripts support the following SDF geometry types:
- **Box** - Rectangular boxes with arbitrary dimensions
- **Cylinder** - Cylindrical objects (pipes, cables, etc.)
- **Sphere** - Spherical objects

Each geometry type generates surface points that represent the obstacle boundaries.

## Pointcloud Density

The `point_density` parameter controls the spacing between generated points:
- `0.05` - Very dense (5cm) - Good for detailed path planning
- `0.1` - Dense (10cm) - Good balance of detail and performance
- `0.15` - Medium (15cm) - Default, good for visualization
- `0.2` - Coarse (20cm) - Fast generation, less detail

## Applications

The generated pointclouds can be used for:
- **Path Planning** - 3D obstacle avoidance algorithms
- **SLAM** - Simultaneous Localization and Mapping
- **Collision Detection** - Real-time safety systems
- **Visualization** - 3D environment representation
- **ML Training** - Training data for navigation algorithms

## Example Output

For the warehouse environment, typical output includes:
- ~15,000-50,000 points (depending on density)
- Coverage of all walls, shelving units, equipment, and obstacles
- 3D representation suitable for drone navigation planning

## Integration with ROS

The generated pointcloud files can be easily integrated with ROS:

```python
# Example ROS integration (pseudo-code)
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np

# Load pointcloud
points = np.loadtxt('warehouse_challenging.txt')

# Convert to ROS PointCloud2 message
# (use pcl_ros or similar for conversion)
```

## Troubleshooting

### No Display Available
If running in a headless environment, visualization may fail. Use `--no-visualize` or catch the exception.

### Memory Issues
For very dense pointclouds, reduce the density parameter or process in chunks.

### SDF Parsing Errors
Ensure the SDF file is valid XML and contains supported geometry types.