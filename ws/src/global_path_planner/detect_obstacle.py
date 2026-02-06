import open3d as o3d
import numpy as np
import os


def _preprocess_point_cloud(pcd, voxel_size=0.05, outlier_removal=True):
    """Helper: Load and preprocess point cloud"""
    pcd_clean = pcd.remove_non_finite_points()
    pcd_down = pcd_clean.voxel_down_sample(voxel_size=voxel_size)
    
    if outlier_removal:
        pcd_filtered, _ = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    else:
        pcd_filtered = pcd_down
    
    return pcd_clean, pcd_filtered


def _compute_plane_normal_angle(plane_normal, expected_normal):
    """Helper: Compute angle between plane normal and expected normal"""
    expected = np.array(expected_normal) / np.linalg.norm(expected_normal)
    cosine_sim = np.abs(np.dot(plane_normal, expected))
    angle_deg = np.degrees(np.arccos(np.clip(cosine_sim, -1, 1)))
    return cosine_sim, angle_deg


def _separate_ground_obstacles(original_pcd, plane_model, distance_threshold):
    """Helper: Separate ground and obstacle points from original cloud"""
    original_points = np.asarray(original_pcd.points)
    original_colors = np.asarray(original_pcd.colors)
    
    [a, b, c, d] = plane_model
    numerator = np.abs(a * original_points[:, 0] + b * original_points[:, 1] + 
                       c * original_points[:, 2] + d)
    denominator = np.sqrt(a**2 + b**2 + c**2)
    distances = numerator / denominator
    
    ground_threshold = distance_threshold * 1.5
    is_ground = distances < ground_threshold
    
    ground_indices = np.where(is_ground)[0]
    obstacle_indices = np.where(~is_ground)[0]
    
    # Create clouds
    ground_cloud = o3d.geometry.PointCloud()
    if len(ground_indices) > 0:
        ground_cloud.points = o3d.utility.Vector3dVector(original_points[ground_indices])
        if original_colors.shape[0] > 0:
            ground_cloud.colors = o3d.utility.Vector3dVector(original_colors[ground_indices])
    
    obstacle_cloud = o3d.geometry.PointCloud()
    if len(obstacle_indices) > 0:
        obstacle_cloud.points = o3d.utility.Vector3dVector(original_points[obstacle_indices])
        if original_colors.shape[0] > 0:
            obstacle_cloud.colors = o3d.utility.Vector3dVector(original_colors[obstacle_indices])
    
    return ground_cloud, obstacle_cloud, ground_indices, obstacle_indices


def _save_results(ground_cloud, obstacle_cloud, original_clean, ground_down, obstacle_down,
                   plane_model, angle_degrees, distance_threshold, ply_file_path, output_prefix):
    """Helper: Save processing results to files"""
    print("\n" + "=" * 60)
    print("SAVING RESULTS")
    print("=" * 60)
    
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    
    # Save point clouds
    o3d.io.write_point_cloud(f"{output_prefix}_obstacles_ground_removed.ply", obstacle_cloud)
    print(f"✓ Saved obstacle point cloud to: {output_prefix}_obstacles_ground_removed.ply")
    
    o3d.io.write_point_cloud(f"{output_prefix}_ground.ply", ground_cloud)
    print(f"✓ Saved ground point cloud to: {output_prefix}_ground.ply")
    
    o3d.io.write_point_cloud(f"{output_prefix}_cleaned.ply", original_clean)
    print(f"✓ Saved cleaned original point cloud to: {output_prefix}_cleaned.ply")
    
    vis_cloud = ground_down + obstacle_down
    o3d.io.write_point_cloud(f"{output_prefix}_visualization.ply", vis_cloud)
    print(f"✓ Saved visualization point cloud to: {output_prefix}_visualization.ply")
    
    # Save statistics
    with open(f"{output_prefix}_statistics.txt", 'w') as f:
        f.write("Ground Plane Detection Results\n" + "=" * 40 + "\n\n")
        f.write(f"Input file: {ply_file_path}\n")
        f.write(f"Cleaned points: {len(original_clean.points)}\n")
        f.write(f"Ground points: {len(ground_cloud.points)}\n")
        f.write(f"Obstacle points: {len(obstacle_cloud.points)}\n")
        if len(original_clean.points) > 0:
            f.write(f"Ground percentage: {(len(ground_cloud.points)/len(original_clean.points))*100:.2f}%\n\n")
        f.write(f"Plane equation: {a:.6f}x + {b:.6f}y + {c:.6f}z + {d:.6f} = 0\n")
        f.write(f"Plane normal: [{plane_normal[0]:.6f}, {plane_normal[1]:.6f}, {plane_normal[2]:.6f}]\n")
        f.write(f"Angle with expected normal: {angle_degrees:.2f}°\n")
    
    print(f"✓ Saved statistics to: {output_prefix}_statistics.txt")


def identify_and_remove_ground(ply_file_path, expected_normal=(0, 0, 1), 
                              normal_tolerance=0.3, vertical_tolerance=15.0,
                              save_obstacles=True, output_prefix=None):
    """
    Identify ground plane and remove it from the original point cloud
    
    Parameters:
    - ply_file_path: Path to the PLY file
    - expected_normal: Expected normal vector for ground (default: Z-up [0,0,1])
    - normal_tolerance: Cosine similarity tolerance for normal verification
    - vertical_tolerance: Angle tolerance in degrees for vertical alignment
    - save_obstacles: Whether to save obstacle point cloud
    - output_prefix: Prefix for output files (if None, uses input filename)
    
    Returns:
    - ground_down: Ground cloud (downsampled)
    - obstacle_down: Obstacle cloud (downsampled)
    - ground_cloud: Full-resolution ground cloud
    - obstacle_cloud: Full-resolution obstacle cloud
    - plane_model: Plane equation [a, b, c, d]
    - is_horizontal: Whether the detected plane is horizontal
    """
    
    if output_prefix is None:
        output_prefix = os.path.splitext(os.path.basename(ply_file_path))[0]
    
    print("=" * 60)
    print("LOADING AND PREPROCESSING")
    print("=" * 60)
    
    # Load and preprocess
    original_pcd = o3d.io.read_point_cloud(ply_file_path)
    original_size = len(original_pcd.points)
    print(f"Original point cloud size: {original_size}")
    
    original_clean, pcd_filtered = _preprocess_point_cloud(original_pcd, voxel_size=0.05)
    print(f"After preprocessing: {len(pcd_filtered.points)} points")
    
    # Estimate normals and detect plane
    print("\n" + "=" * 60)
    print("GROUND PLANE DETECTION")
    print("=" * 60)
    
    pcd_filtered.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    
    distance_threshold = 0.4
    plane_model, inliers_down = pcd_filtered.segment_plane(
        distance_threshold=distance_threshold, ransac_n=3, num_iterations=2000
    )
    
    [a, b, c, d] = plane_model
    plane_normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    
    print(f"Detected plane equation: {a:.6f}x + {b:.6f}y + {c:.6f}z + {d:.6f} = 0")
    print(f"Plane normal vector: [{plane_normal[0]:.6f}, {plane_normal[1]:.6f}, {plane_normal[2]:.6f}]")
    
    # Verify normal
    cosine_sim, angle_degrees = _compute_plane_normal_angle(plane_normal, expected_normal)
    print(f"Cosine similarity: {cosine_sim:.4f}, Angle: {angle_degrees:.2f}°")
    
    is_horizontal = cosine_sim > (1 - normal_tolerance)
    is_vertical_aligned = angle_degrees < vertical_tolerance
    
    if is_horizontal and is_vertical_aligned:
        print("✓ Verified: Detected plane appears to be ground (horizontal plane)")
    else:
        print("✗ Warning: Detected plane doesn't match expected ground orientation")
    
    # Separate ground and obstacles
    print("\n" + "=" * 60)
    print("SEPARATING GROUND FROM OBSTACLES")
    print("=" * 60)
    
    ground_cloud, obstacle_cloud, g_idx, o_idx = _separate_ground_obstacles(
        original_clean, plane_model, distance_threshold
    )
    print(f"Ground points: {len(ground_cloud.points)}")
    print(f"Obstacle points: {len(obstacle_cloud.points)}")
    
    # Prepare visualizations
    ground_down = pcd_filtered.select_by_index(inliers_down)
    obstacle_down = pcd_filtered.select_by_index(inliers_down, invert=True)
    
    for cloud in [ground_cloud, obstacle_cloud, ground_down, obstacle_down]:
        if cloud == ground_cloud or cloud == ground_down:
            cloud.paint_uniform_color([0.2, 0.8, 0.2])  # Green
        else:
            cloud.paint_uniform_color([0.3, 0.3, 1.0])  # Blue
    
    # Save results
    if save_obstacles:
        _save_results(ground_cloud, obstacle_cloud, original_clean, ground_down, 
                     obstacle_down, plane_model, angle_degrees, distance_threshold,
                     ply_file_path, output_prefix)
    
    # Print statistics
    print("\n" + "=" * 60)
    print("STATISTICS")
    print("=" * 60)
    
    return ground_down, obstacle_down, ground_cloud, obstacle_cloud, plane_model, is_horizontal


def visualize_ground_removal(ground_down, obstacle_down, ground_full, obstacle_full):
    """Visualize both downsampled and full-resolution results"""
    print("\n" + "=" * 60)
    print("VISUALIZATION")
    print("=" * 60)
    
    for vis_name, clouds in [
        ("Downsampled Segmentation", [ground_down, obstacle_down]),
        ("Full Resolution (Ground Removed)", [obstacle_full])
    ]:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=vis_name, width=800, height=600)
        for cloud in clouds:
            vis.add_geometry(cloud)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        vis.run()
        vis.destroy_window()


def verify_ground_removal(ground_cloud, obstacle_cloud, plane_model, threshold=0.05):
    """Verify that ground has been properly removed from obstacle cloud"""
    print("\n" + "=" * 60)
    print("VERIFYING GROUND REMOVAL")
    print("=" * 60)
    
    [a, b, c, d] = plane_model
    
    if len(obstacle_cloud.points) == 0:
        print("No obstacle points to verify!")
        return
    
    obstacle_points = np.asarray(obstacle_cloud.points)
    numerator = np.abs(a * obstacle_points[:, 0] + b * obstacle_points[:, 1] + 
                       c * obstacle_points[:, 2] + d)
    denominator = np.sqrt(a**2 + b**2 + c**2)
    distances = numerator / denominator
    
    possible_ground = np.sum(distances < threshold)
    if possible_ground > 0:
        pct = (possible_ground/len(obstacle_points)*100)
        print(f"⚠ {possible_ground} obstacle points ({pct:.2f}%) near ground plane")
        if pct > 5:
            print("  Consider using a smaller distance threshold!")
    else:
        print("✓ Good: No obstacle points detected near ground plane")
    
    if len(ground_cloud.points) > 0:
        ground_points = np.asarray(ground_cloud.points)
        numerator_g = np.abs(a * ground_points[:, 0] + b * ground_points[:, 1] + 
                             c * ground_points[:, 2] + d)
        distances_g = numerator_g / denominator
        print(f"Ground avg/max distance to plane: {np.mean(distances_g):.6f}/{np.max(distances_g):.6f}")


def main():
    """Main execution: identify and remove ground from point cloud"""
    ply_file = "filtered_max_height.ply"
    
    print("=" * 60)
    print("GROUND REMOVAL FROM POINT CLOUD")
    print("=" * 60)
    
    try:
        ground_down, obstacle_down, ground_full, obstacle_full, plane_eq, is_valid = identify_and_remove_ground(
            ply_file,
            save_obstacles=True,
            output_prefix="processed"
        )
        
        if obstacle_full is not None and len(obstacle_full.points) > 0:
            print("\nSuccessfully removed ground from point cloud!")
            verify_ground_removal(ground_full, obstacle_full, plane_eq)
            visualize_ground_removal(ground_down, obstacle_down, ground_full, obstacle_full)
        else:
            print("No obstacle points found or error in processing!")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
