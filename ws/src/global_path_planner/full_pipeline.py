#!/usr/bin/env python3
"""
Complete Pipeline: Raw Point Cloud → Preprocessed Map → Global Path → Optimized Trajectory

This script orchestrates the full planning pipeline:
1. Point cloud preprocessing (ground removal, filtering, traversability mapping)
2. Global path planning using A* on graph
3. Trajectory generation using Pure Pursuit 3D controller
"""

import argparse
import glob
import os
import time
from typing import List, Tuple

import numpy as np
import open3d as o3d

from preprocess_pipeline import run_pipeline
from graph_a_star_core import GlobalPlanner, build_waypoints_from_path
from pure_pursuit3D_multi_model_core import OmnidirectionalPurePursuit3D, Waypoint


def run_full_pipeline(
    input_ply: str,
    output_dir: str,
    start_coord: Tuple[float, float, float],
    goal_coord: Tuple[float, float, float],
    start_yaw: float = 0.0,
    goal_yaw: float = 0.0,
    use_identify_ground: bool = False,
    xy_resolution: float = 0.05,
    map_resolution: float = 0.5,
    height_diff_thresh: float = 0.05,
    slope_angle_thresh: float = 20.0,
    max_z: float = 3.0,
    lookahead_dist: float = 0.5,
    v_max: float = 1.0,
    omega_max: float = 1.0,
    dt: float = 0.1,
    visualize: bool = True
) -> Tuple[List[np.ndarray], List[np.ndarray], List[float]]:
    """
    Execute the complete planning pipeline.
    
    Parameters:
    -----------
    input_ply : str
        Path to input point cloud file
    output_dir : str
        Directory for output files
    start_coord : tuple
        Start position (x, y, z)
    goal_coord : tuple
        Goal position (x, y, z)
    start_yaw : float
        Initial heading in radians
    goal_yaw : float
        Final heading in radians
    use_identify_ground : bool
        Use identify_ground instead of detect_obstacle
    xy_resolution : float
        Grid resolution for max height filtering
    map_resolution : float
        Traversability map resolution
    height_diff_thresh : float
        Height difference threshold for obstacles
    slope_angle_thresh : float
        Slope angle threshold in degrees
    max_z : float
        Maximum height to consider (ceiling removal)
    lookahead_dist : float
        Pure Pursuit lookahead distance
    v_max : float
        Maximum linear velocity (m/s)
    omega_max : float
        Maximum angular velocity (rad/s)
    dt : float
        Time step for trajectory simulation
    visualize : bool
        Enable visualizations
    
    Returns:
    --------
    path_waypoints : List[np.ndarray]
        Global path waypoints
    trajectory_positions : List[np.ndarray]
        Executed trajectory positions
    trajectory_times : List[float]
        Time stamps for trajectory
    """
    
    overall_start_time = time.time()
    
    # ========== STEP 1: PREPROCESS POINT CLOUD ==========
    print("\n" + "=" * 70)
    print("STEP 1: POINT CLOUD PREPROCESSING")
    print("=" * 70)
    
    preprocess_start = time.time()
    
    cfree_ply, cobs_ply = run_pipeline(
        input_ply=input_ply,
        output_dir=output_dir,
    )
    
    preprocess_time = time.time() - preprocess_start
    
    print(f"\n Path preprocessing complete in {preprocess_time:.2f}s")
    print(f"  - Cfree (navigable): {cfree_ply}")
    print(f"  - Cobs (obstacles): {cobs_ply}")
    
    # Navigable map already from preprocessing
    navigable_ply = cfree_ply
    print(f"  - Navigable map: {navigable_ply}")
    
    
    # ========== STEP 2: GLOBAL PATH PLANNING ==========
    print("\n" + "=" * 70)
    print("STEP 2: GLOBAL PATH PLANNING (A*)")
    print("=" * 70)
    
    planning_start = time.time()
    
    # Initialize planner
    print("Initializing graph-based planner...")
    planner = GlobalPlanner(navigable_ply, cobs_ply, voxel_size=0.3)
    
    # Find nearest graph nodes to start/goal
    start_index = planner.find_nearest_point_index(start_coord)
    goal_index = planner.find_nearest_point_index(goal_coord)
    
    print(f"Start node: {start_index} at {planner.pcd_pts[start_index]}")
    print(f"Goal node:  {goal_index} at {planner.pcd_pts[goal_index]}")
    
    # Run A* search
    print("\nSearching for optimal path...")
    path_indices = planner.astar(start_index, goal_index, start_yaw, goal_yaw)
    
    planning_time = time.time() - planning_start
    
    if not path_indices:
        print("⚠ A* failed to find a path! Creating linear interpolation fallback...")
        # Create linear interpolation as fallback
        start_pt = planner.pcd_pts[start_index]
        goal_pt = planner.pcd_pts[goal_index]
        
        # Interpolate with 10 points
        path_indices = [start_index]
        for i in range(1, 11):
            t = i / 10.0
            interp_pt = (1 - t) * start_pt + t * goal_pt
            # Find nearest point in cloud to this interpolation
            distances = np.linalg.norm(planner.pcd_pts - interp_pt, axis=1)
            nearest_idx = np.argmin(distances)
            if nearest_idx not in path_indices:
                path_indices.append(nearest_idx)
        if goal_index not in path_indices:
            path_indices.append(goal_index)
        
        # Convert to (node_idx, yaw) tuples
        path_indices = [(idx, 0.0) for idx in path_indices]
    
    print(f"\n Path planning complete in {planning_time:.2f}s")
    print(f"  - Path length: {len(path_indices)} waypoints")
    
    # Convert indices to Waypoint objects
    waypoints = build_waypoints_from_path(planner, path_indices, goal_yaw)
    waypoints_position = np.array([wp.position for wp in waypoints])
    
    # Calculate path length
    path_length = sum(
        np.linalg.norm(waypoints_position[i+1] - waypoints_position[i])
        for i in range(len(waypoints_position) - 1)
    )
    print(f"  - Total path length: {path_length:.2f}m")
    
    # Visualize if requested
    if visualize:
        try:
            planner.visualize_scene_with_orientation_robust(path_indices, start_index, goal_index)
        except Exception as e:
            print(f"Warning: Visualization failed - {e}")
    
    
    # ========== STEP 3: TRAJECTORY GENERATION ==========
    print("\n" + "=" * 70)
    print("STEP 3: TRAJECTORY GENERATION (Pure Pursuit 3D)")
    print("=" * 70)
    
    # Configure Pure Pursuit controller
    trajectory_start = time.time()
    params = {
        'lookahead_dist': lookahead_dist,
        'k_v': 1.0,
        'k_omega': 1.0,
        'v_max': v_max,
        'omega_max': omega_max,
        'v_omni_max': v_max * 1.5,
        'angle_thresh_deg': 30,
        'z_threshold': 0.05,
        'weight_time': True,
        'dt': dt,
        'max_time': 60.0,
        'goal_threshold': 0.1,
        'visualize': False,
    }
    
    print(f"Controller parameters:")
    print(f"  - Lookahead: {lookahead_dist}m")
    print(f"  - Max velocity: {v_max}m/s")
    print(f"  - Max angular velocity: {omega_max}rad/s")
    print(f"  - Time step: {dt}s")
    
    # Initialize and run controller
    print("\nExecuting trajectory...")
    controller = OmnidirectionalPurePursuit3D(waypoints=waypoints, params=params)
    controller.run()
    
    trajectory_time = time.time() - trajectory_start
    
    # Get trajectory results
    trajectory_positions = [np.array([x, y, z]) for x, y, z in zip(
        controller.history['x'],
        controller.history['y'],
        controller.history['z']
    )]
    trajectory_times = controller.history['time']
    
    final_pos = trajectory_positions[-1]
    goal_pos = waypoints[-1].position
    final_error = np.linalg.norm(final_pos - goal_pos)
    
    print(f"\n✓ Trajectory generation complete in {trajectory_time:.2f}s")
    print(f"  - Trajectory points: {len(trajectory_positions)}")
    print(f"  - Execution time: {trajectory_times[-1]:.2f}s")
    print(f"  - Final position: [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}]")
    print(f"  - Goal position:  [{goal_pos[0]:.2f}, {goal_pos[1]:.2f}, {goal_pos[2]:.2f}]")
    print(f"  - Final error: {final_error:.3f}m")
    print(f"  - Goal reached: {'✓' if final_error < params['goal_threshold'] else '✗'}")
    
    # Save trajectory to CSV
    trajectory_csv = os.path.join(output_dir, f"trajectory_{os.path.basename(input_ply).replace('.ply', '')}.csv")
    with open(trajectory_csv, 'w') as f:
        f.write("time,x,y,z\n")
        for t, pos in zip(trajectory_times, trajectory_positions):
            f.write(f"{t:.4f},{pos[0]:.6f},{pos[1]:.6f},{pos[2]:.6f}\n")
    print(f"\n✓ Trajectory saved: {trajectory_csv}")
    
    # Visualize trajectory
    if visualize:
        try:
            controller.plot_3d()
        except Exception as e:
            print(f"Warning: Trajectory visualization failed - {e}")
    
    
    # ========== SUMMARY ==========
    total_time = time.time() - overall_start_time
    
    print("\n" + "=" * 70)
    print("PIPELINE COMPLETE")
    print("=" * 70)
    print(f"Total execution time: {total_time:.2f}s")
    print(f"  1. Preprocessing:  {preprocess_time:.2f}s ({preprocess_time/total_time*100:.1f}%)")
    print(f"  2. Path planning:  {planning_time:.2f}s ({planning_time/total_time*100:.1f}%)")
    print(f"  3. Trajectory gen: {trajectory_time:.2f}s ({trajectory_time/total_time*100:.1f}%)")
    print()
    print(f"Results:")
    print(f"  - Global path: {len(waypoints)} waypoints, {path_length:.2f}m")
    print(f"  - Trajectory: {len(trajectory_positions)} poses, {trajectory_times[-1]:.2f}s")
    print(f"  - Goal reached: {'Yes' if final_error < params['goal_threshold'] else 'No'} (error: {final_error:.3f}m)")
    
    return waypoints, trajectory_positions, trajectory_times


def main():
    """Main entry point with command-line argument parsing"""
    parser = argparse.ArgumentParser(
        description="Complete planning pipeline: Point Cloud → Trajectory",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Input/Output
    parser.add_argument('--input', '-i', type=str, default='filtered_max_height.ply',
                        help='Input point cloud file')
    parser.add_argument('--output', '-o', type=str, default='output',
                        help='Output directory')
    
    # Start/Goal
    parser.add_argument('--start', nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        help='Start position (x y z)')
    parser.add_argument('--goal', nargs=3, type=float, default=[5.0, 5.0, 0.5],
                        help='Goal position (x y z)')
    parser.add_argument('--start-yaw', type=float, default=0.0,
                        help='Start yaw angle in radians')
    parser.add_argument('--goal-yaw', type=float, default=0.0,
                        help='Goal yaw angle in radians')
    
    # Preprocessing
    parser.add_argument('--use-identify-ground', action='store_true',
                        help='Use identify_ground instead of detect_obstacle')
    parser.add_argument('--xy-resolution', type=float, default=0.05,
                        help='Grid resolution for max height filtering')
    parser.add_argument('--map-resolution', type=float, default=0.5,
                        help='Traversability map resolution')
    parser.add_argument('--height-thresh', type=float, default=0.05,
                        help='Height difference threshold')
    parser.add_argument('--slope-thresh', type=float, default=20.0,
                        help='Slope angle threshold (degrees)')
    parser.add_argument('--max-z', type=float, default=3.0,
                        help='Maximum height (ceiling removal)')
    
    # Controller
    parser.add_argument('--lookahead', type=float, default=0.5,
                        help='Pure Pursuit lookahead distance')
    parser.add_argument('--v-max', type=float, default=1.0,
                        help='Maximum linear velocity (m/s)')
    parser.add_argument('--omega-max', type=float, default=1.0,
                        help='Maximum angular velocity (rad/s)')
    parser.add_argument('--dt', type=float, default=0.1,
                        help='Time step for simulation (s)')
    
    # Visualization
    parser.add_argument('--no-viz', action='store_true',
                        help='Disable visualizations')
    
    args = parser.parse_args()
    
    # Run pipeline
    try:
        run_full_pipeline(
            input_ply=args.input,
            output_dir=args.output,
            start_coord=tuple(args.start),
            goal_coord=tuple(args.goal),
            start_yaw=args.start_yaw,
            goal_yaw=args.goal_yaw,
            use_identify_ground=args.use_identify_ground,
            xy_resolution=args.xy_resolution,
            map_resolution=args.map_resolution,
            height_diff_thresh=args.height_thresh,
            slope_angle_thresh=args.slope_thresh,
            max_z=args.max_z,
            lookahead_dist=args.lookahead,
            v_max=args.v_max,
            omega_max=args.omega_max,
            dt=args.dt,
            visualize=not args.no_viz
        )
    except Exception as e:
        print(f"\n Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
