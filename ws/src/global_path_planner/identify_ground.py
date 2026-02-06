import open3d as o3d
import numpy as np
import os


def _preprocess_cloud(pcd, voxel_size=0.1):
    """Helper: Standard preprocessing pipeline"""
    pcd_clean = pcd.remove_non_finite_points()
    pcd_down = pcd_clean.voxel_down_sample(voxel_size=voxel_size)
    pcd_filtered, _ = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return pcd_clean, pcd_filtered


def _verify_plane_orientation(plane_normal, expected_normal, normal_tolerance, vertical_tolerance):
    """Helper: Verify if plane normal matches expected orientation"""
    expected = np.array(expected_normal) / np.linalg.norm(expected_normal)
    cos_sim = np.abs(np.dot(plane_normal, expected))
    angle_deg = np.degrees(np.arccos(np.clip(cos_sim, -1, 1)))
    
    is_horizontal = cos_sim > (1 - normal_tolerance)
    is_vertical = angle_deg < vertical_tolerance
    
    return is_horizontal and is_vertical, angle_deg, cos_sim


def identify_ground_plane(ply_file_path, expected_normal=(0, 0, 1), 
                         normal_tolerance=0.3, vertical_tolerance=15.0,
                         save_obstacles=True, output_prefix=None):
    """
    Identify ground plane in a PLY file with preprocessing and normal verification
    """
    
    if output_prefix is None:
        output_prefix = os.path.splitext(os.path.basename(ply_file_path))[0]
    
    # Load and preprocess
    print("Loading and preprocessing point cloud...")
    pcd = o3d.io.read_point_cloud(ply_file_path)
    print(f"Original size: {len(pcd.points)}")
    
    original_pcd, pcd_filtered = _preprocess_cloud(pcd, voxel_size=0.1)
    print(f"Filtered size: {len(pcd_filtered.points)}")
    
    # Detect plane
    print("Performing RANSAC plane segmentation...")
    pcd_filtered.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    
    distance_threshold = 0.4
    plane_model, inliers = pcd_filtered.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=2000
    )
    
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    
    print(f"Plane: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
    print(f"Normal: [{plane_normal[0]:.4f}, {plane_normal[1]:.4f}, {plane_normal[2]:.4f}]")
    
    # Verify orientation
    is_valid, angle_deg, cos_sim = _verify_plane_orientation(plane_normal, expected_normal, 
                                                             normal_tolerance, vertical_tolerance)
    print(f"Cosine similarity: {cos_sim:.4f}, Angle: {angle_deg:.2f}°")
    
    if is_valid:
        print("✓ Verified: Detected plane appears to be ground (horizontal)")
    else:
        print("✗ Warning: Plane doesn't match expected ground orientation")
    
    # Separate ground and obstacles
    ground_cloud = pcd_filtered.select_by_index(inliers)
    non_ground_cloud = pcd_filtered.select_by_index(inliers, invert=True)
    
    ground_pct = (len(ground_cloud.points) / len(pcd_filtered.points)) * 100
    print(f"\nGround points: {len(ground_cloud.points)} ({ground_pct:.1f}%)")
    print(f"Obstacle points: {len(non_ground_cloud.points)}")
    
    # Color code
    ground_cloud.paint_uniform_color([0.2, 0.8, 0.2])
    non_ground_cloud.paint_uniform_color([0.3, 0.3, 1.0])
    
    # Save if requested
    if save_obstacles:
        o3d.io.write_point_cloud(f"{output_prefix}_obstacles.ply", non_ground_cloud)
        print(f"✓ Saved obstacles to: {output_prefix}_obstacles.ply")
        
        o3d.io.write_point_cloud(f"{output_prefix}_ground.ply", ground_cloud)
        print(f"✓ Saved ground to: {output_prefix}_ground.ply")
        
        segmented = ground_cloud + non_ground_cloud
        o3d.io.write_point_cloud(f"{output_prefix}_segmented.ply", segmented)
        print(f"✓ Saved segmented to: {output_prefix}_segmented.ply")
    
    return ground_cloud, non_ground_cloud, plane_model, is_valid


def extract_full_resolution_obstacles(original_pcd, plane_model, distance_threshold=0.4):
    """Extract obstacles from full-resolution point cloud using detected plane"""
    print("Extracting full-resolution obstacles...")
    
    pcd_clean = original_pcd.remove_non_finite_points()
    _, inliers_full = pcd_clean.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=2000
    )
    
    ground_full = pcd_clean.select_by_index(inliers_full)
    obstacles_full = pcd_clean.select_by_index(inliers_full, invert=True)
    
    print(f"Full-resolution ground: {len(ground_full.points)}")
    print(f"Full-resolution obstacles: {len(obstacles_full.points)}")
    
    return ground_full, obstacles_full


def iterative_ground_detection(ply_file_path, max_iterations=5, save_obstacles=True, output_prefix=None):
    """Iteratively detect planes until finding one matching ground orientation"""
    print("=== ITERATIVE GROUND DETECTION ===")
    
    if output_prefix is None:
        output_prefix = os.path.splitext(os.path.basename(ply_file_path))[0]
    
    original_pcd = o3d.io.read_point_cloud(ply_file_path)
    _, remaining_cloud = _preprocess_cloud(o3d.io.read_point_cloud(ply_file_path), voxel_size=0.05)
    
    expected_normal = np.array([0, 0, 1])
    distance_threshold = 0.05
    
    for i in range(max_iterations):
        if len(remaining_cloud.points) < 100:
            break
        
        print(f"\nIteration {i+1}: {len(remaining_cloud.points)} points")
        
        remaining_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000
        )
        
        [a, b, c, d] = plane_model
        plane_normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        
        cos_sim = np.abs(np.dot(plane_normal, expected_normal))
        angle_deg = np.degrees(np.arccos(np.clip(cos_sim, -1, 1)))
        
        print(f"Plane angle: {angle_deg:.2f}°")
        
        if angle_deg < 15.0:  # Within 15 degrees of vertical
            print(f"✓ Found ground plane at iteration {i+1}")
            ground_cloud = remaining_cloud.select_by_index(inliers)
            non_ground_cloud = remaining_cloud.select_by_index(inliers, invert=True)
            
            if save_obstacles:
                o3d.io.write_point_cloud(f"{output_prefix}_obstacles_iterative.ply", non_ground_cloud)
                ground_full, obstacles_full = extract_full_resolution_obstacles(original_pcd, plane_model, distance_threshold)
                o3d.io.write_point_cloud(f"{output_prefix}_obstacles_full_resolution.ply", obstacles_full)
                return ground_cloud, non_ground_cloud, plane_model, obstacles_full
            
            return ground_cloud, non_ground_cloud, plane_model, None
        
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
    
    print("No suitable ground plane found")
    return None, None, None, None


def visualize_results(ground_cloud, non_ground_cloud, original_cloud=None):
    """Visualize the segmentation results"""
    geometries = [ground_cloud, non_ground_cloud]
    if original_cloud is not None:
        geometries.append(original_cloud)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for geom in geometries:
        vis.add_geometry(geom)
    
    opt = vis.get_render_option()
    opt.background_color = np.array([0.1, 0.1, 0.1])
    opt.point_size = 2.0
    
    vis.run()
    vis.destroy_window()


def main():
    """Main execution: ground plane detection with obstacle extraction"""
    ply_file = "filtered_max_height.ply"
    
    print("=== METHOD 1: Direct Ground Detection ===")
    ground, obstacles, plane_eq, is_valid = identify_ground_plane(
        ply_file, 
        save_obstacles=True,
        output_prefix="output"
    )
    
    if is_valid:
        visualize_results(ground, obstacles)
        print("\n=== Extracting full-resolution obstacles ===")
        original_pcd = o3d.io.read_point_cloud(ply_file)
        ground_full, obstacles_full = extract_full_resolution_obstacles(original_pcd, plane_eq)
        o3d.io.write_point_cloud("obstacles_full_resolution.ply", obstacles_full)
        print("✓ Saved full-resolution obstacle point cloud")
    else:
        print("\n=== METHOD 2: Iterative Detection ===")
        ground_iter, obstacles_iter, plane_eq_iter, obstacles_full_iter = iterative_ground_detection(
            ply_file,
            save_obstacles=True,
            output_prefix="output_iterative"
        )
        
        if ground_iter is not None:
            ground_iter.paint_uniform_color([0.2, 0.8, 0.2])
            obstacles_iter.paint_uniform_color([0.3, 0.3, 1.0])
            visualize_results(ground_iter, obstacles_iter)


if __name__ == "__main__":
    main()