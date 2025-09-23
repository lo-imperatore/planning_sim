#!/usr/bin/env python3
"""
Simple example script to generate pointcloud from the warehouse SDF file
"""

import sys
import os
import numpy as np

# Add the scripts directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from sdf_to_pointcloud import SDFParser, PointcloudGenerator, PointcloudVisualizer


def generate_warehouse_pointcloud():
    """Generate pointcloud from the warehouse SDF file"""
    
    # Path to the warehouse SDF file
    warehouse_sdf = "/work/ws/src/uav_gz_demo/worlds/x3_warehouse_challenging.sdf"
    
    if not os.path.exists(warehouse_sdf):
        print(f"Error: Warehouse SDF file not found at {warehouse_sdf}")
        return
    
    print("=== Warehouse Pointcloud Generator ===")
    print(f"Processing: {warehouse_sdf}")
    
    # Parse the SDF file
    print("\n1. Parsing SDF file...")
    parser = SDFParser(warehouse_sdf)
    geometries = parser.parse()
    
    print(f"   Found {len(geometries)} geometric objects:")
    for i, geom in enumerate(geometries):
        print(f"   {i+1}. {geom.model_name}/{geom.link_name}: {geom.geometry_type}")
    
    # Generate pointcloud with medium density
    print("\n2. Generating pointcloud...")
    generator = PointcloudGenerator(point_density=0.2)  # 20cm between points
    pointcloud = generator.generate_pointcloud(geometries)
    
    print(f"   Generated {len(pointcloud)} points")
    
    if len(pointcloud) == 0:
        print("   Warning: No points generated!")
        return
    
    # Print pointcloud statistics
    print(f"\n3. Pointcloud Statistics:")
    print(f"   X range: {pointcloud[:, 0].min():.2f} to {pointcloud[:, 0].max():.2f} m")
    print(f"   Y range: {pointcloud[:, 1].min():.2f} to {pointcloud[:, 1].max():.2f} m")
    print(f"   Z range: {pointcloud[:, 2].min():.2f} to {pointcloud[:, 2].max():.2f} m")
    
    # Save pointcloud files
    print("\n4. Saving pointcloud files...")
    output_dir = "/work/ws/src/uav_gz_demo/pointclouds"
    os.makedirs(output_dir, exist_ok=True)
    
    base_filename = os.path.join(output_dir, "warehouse_challenging")
    
    # Save in different formats
    PointcloudVisualizer.save_pointcloud_txt(pointcloud, base_filename + ".txt")
    PointcloudVisualizer.save_pointcloud_ply(pointcloud, base_filename + ".ply")
    
    # Generate visualization
    print("\n5. Creating visualization...")
    try:
        PointcloudVisualizer.plot_pointcloud(
            pointcloud, 
            title="Warehouse Challenging - Pointcloud",
            point_size=2.0,
            save_path=base_filename + "_visualization.png"
        )
    except Exception as e:
        print(f"   Visualization failed: {e}")
        print("   (This might happen in headless environments)")
    
    print(f"\n✓ Complete! Files saved in: {output_dir}")
    
    return pointcloud


def generate_dense_pointcloud():
    """Generate a denser pointcloud for detailed navigation planning"""
    
    warehouse_sdf = "/work/ws/src/uav_gz_demo/worlds/x3_warehouse_challenging.sdf"
    
    print("=== Dense Warehouse Pointcloud Generator ===")
    print("Generating high-density pointcloud for path planning...")
    
    # Parse the SDF file
    parser = SDFParser(warehouse_sdf)
    geometries = parser.parse()
    
    # Generate dense pointcloud (5cm between points)
    generator = PointcloudGenerator(point_density=0.05)
    pointcloud = generator.generate_pointcloud(geometries)
    
    print(f"Generated dense pointcloud with {len(pointcloud)} points")
    
    # Save dense pointcloud
    output_dir = "/home/pietro/planning_sim/ws/src/uav_gz_demo/pointclouds"
    os.makedirs(output_dir, exist_ok=True)
    
    base_filename = os.path.join(output_dir, "warehouse_challenging_dense")
    PointcloudVisualizer.save_pointcloud_txt(pointcloud, base_filename + ".txt")
    PointcloudVisualizer.save_pointcloud_ply(pointcloud, base_filename + ".ply")
    
    print(f"Dense pointcloud saved to: {base_filename}.txt and {base_filename}.ply")
    
    return pointcloud


def analyze_navigation_space():
    """Analyze the free space for drone navigation"""
    
    warehouse_sdf = "/work/ws/src/uav_gz_demo/worlds/x3_warehouse_challenging.sdf"
    
    print("=== Navigation Space Analysis ===")
    
    # Parse and generate pointcloud
    parser = SDFParser(warehouse_sdf)
    geometries = parser.parse()
    generator = PointcloudGenerator(point_density=0.1)
    obstacle_points = generator.generate_pointcloud(geometries)
    
    # Define workspace bounds (from SDF analysis)
    workspace_bounds = {
        'x_min': -20, 'x_max': 20,
        'y_min': -15, 'y_max': 15,
        'z_min': 0.1, 'z_max': 5.9  # Flying height range
    }
    
    print(f"Workspace bounds: {workspace_bounds}")
    print(f"Obstacle points: {len(obstacle_points)}")
    
    # Calculate some basic navigation metrics
    total_volume = (workspace_bounds['x_max'] - workspace_bounds['x_min']) * \
                   (workspace_bounds['y_max'] - workspace_bounds['y_min']) * \
                   (workspace_bounds['z_max'] - workspace_bounds['z_min'])
    
    print(f"Total workspace volume: {total_volume:.2f} m³")
    
    # Estimate obstacle density
    if len(obstacle_points) > 0:
        obstacle_density = len(obstacle_points) / total_volume
        print(f"Obstacle point density: {obstacle_density:.2f} points/m³")
    
    return obstacle_points, workspace_bounds


if __name__ == "__main__":
    # Check command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == "dense":
            generate_dense_pointcloud()
        elif sys.argv[1] == "analyze":
            analyze_navigation_space()
        else:
            print("Usage: python warehouse_pointcloud.py [dense|analyze]")
    else:
        # Default: generate standard pointcloud
        generate_warehouse_pointcloud()