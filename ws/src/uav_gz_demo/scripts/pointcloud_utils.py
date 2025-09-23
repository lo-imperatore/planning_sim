#!/usr/bin/env python3
"""
Pointcloud Analysis and Utilities
Additional tools for working with warehouse pointclouds
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

# Add the scripts directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from sdf_to_pointcloud import SDFParser, PointcloudGenerator, PointcloudVisualizer


class PointcloudAnalyzer:
    """Analyze pointcloud data for navigation planning"""
    
    def __init__(self, pointcloud_file: str):
        """Load pointcloud from file"""
        if pointcloud_file.endswith('.txt'):
            self.points = np.loadtxt(pointcloud_file)
        elif pointcloud_file.endswith('.ply'):
            self.points = self._load_ply(pointcloud_file)
        else:
            raise ValueError("Unsupported file format. Use .txt or .ply")
        
        print(f"Loaded {len(self.points)} points from {pointcloud_file}")
    
    def _load_ply(self, ply_file: str) -> np.ndarray:
        """Load points from PLY file"""
        points = []
        with open(ply_file, 'r') as f:
            header_ended = False
            for line in f:
                if header_ended:
                    coords = line.strip().split()
                    if len(coords) >= 3:
                        points.append([float(coords[0]), float(coords[1]), float(coords[2])])
                elif line.strip() == 'end_header':
                    header_ended = True
        return np.array(points)
    
    def get_bounds(self):
        """Get pointcloud bounds"""
        if len(self.points) == 0:
            return None
        
        return {
            'x_min': float(self.points[:, 0].min()),
            'x_max': float(self.points[:, 0].max()),
            'y_min': float(self.points[:, 1].min()),
            'y_max': float(self.points[:, 1].max()),
            'z_min': float(self.points[:, 2].min()),
            'z_max': float(self.points[:, 2].max())
        }
    
    def analyze_density(self, voxel_size: float = 1.0):
        """Analyze point density in voxel grid"""
        bounds = self.get_bounds()
        if bounds is None:
            return None
        
        # Calculate voxel grid dimensions
        nx = int(np.ceil((bounds['x_max'] - bounds['x_min']) / voxel_size))
        ny = int(np.ceil((bounds['y_max'] - bounds['y_min']) / voxel_size))
        nz = int(np.ceil((bounds['z_max'] - bounds['z_min']) / voxel_size))
        
        # Count points per voxel
        voxel_counts = np.zeros((nx, ny, nz))
        
        for point in self.points:
            i = int((point[0] - bounds['x_min']) / voxel_size)
            j = int((point[1] - bounds['y_min']) / voxel_size)
            k = int((point[2] - bounds['z_min']) / voxel_size)
            
            # Clamp indices
            i = max(0, min(i, nx-1))
            j = max(0, min(j, ny-1))
            k = max(0, min(k, nz-1))
            
            voxel_counts[i, j, k] += 1
        
        return {
            'voxel_size': voxel_size,
            'grid_shape': (nx, ny, nz),
            'counts': voxel_counts,
            'occupied_voxels': np.sum(voxel_counts > 0),
            'total_voxels': nx * ny * nz,
            'max_density': float(voxel_counts.max()),
            'mean_density': float(voxel_counts[voxel_counts > 0].mean()) if np.sum(voxel_counts > 0) > 0 else 0
        }
    
    def find_clearance_map(self, height: float = 2.0, grid_resolution: float = 0.2):
        """Generate 2D clearance map at given height"""
        bounds = self.get_bounds()
        if bounds is None:
            return None
        
        # Create 2D grid
        nx = int(np.ceil((bounds['x_max'] - bounds['x_min']) / grid_resolution))
        ny = int(np.ceil((bounds['y_max'] - bounds['y_min']) / grid_resolution))
        
        clearance_map = np.ones((nx, ny)) * float('inf')  # Start with infinite clearance
        
        # For each grid cell, find minimum distance to obstacles
        for i in range(nx):
            for j in range(ny):
                x = bounds['x_min'] + i * grid_resolution
                y = bounds['y_min'] + j * grid_resolution
                
                # Find closest obstacle point at similar height
                height_mask = np.abs(self.points[:, 2] - height) < 1.0  # Within 1m of target height
                nearby_points = self.points[height_mask]
                
                if len(nearby_points) > 0:
                    distances = np.sqrt((nearby_points[:, 0] - x)**2 + (nearby_points[:, 1] - y)**2)
                    clearance_map[i, j] = distances.min()
        
        return {
            'height': height,
            'resolution': grid_resolution,
            'shape': (nx, ny),
            'clearance_map': clearance_map,
            'x_coords': np.linspace(bounds['x_min'], bounds['x_max'], nx),
            'y_coords': np.linspace(bounds['y_min'], bounds['y_max'], ny)
        }
    
    def plot_clearance_map(self, clearance_data, save_path: str = None):
        """Plot 2D clearance map"""
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Clip very large clearance values for better visualization
        clearance_clipped = np.clip(clearance_data['clearance_map'], 0, 5.0)
        
        im = ax.imshow(clearance_clipped.T, 
                      extent=[clearance_data['x_coords'][0], clearance_data['x_coords'][-1],
                             clearance_data['y_coords'][0], clearance_data['y_coords'][-1]],
                      origin='lower', cmap='RdYlBu', aspect='equal')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'Clearance Map at Height {clearance_data["height"]}m')
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Clearance Distance (m)')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Clearance map saved to: {save_path}")
        
        plt.show()
    
    def generate_navigation_waypoints(self, height: float = 2.0, min_clearance: float = 1.5):
        """Generate safe navigation waypoints"""
        clearance_data = self.find_clearance_map(height, grid_resolution=0.5)
        
        # Find cells with sufficient clearance
        safe_mask = clearance_data['clearance_map'] >= min_clearance
        safe_indices = np.where(safe_mask)
        
        waypoints = []
        for i, j in zip(safe_indices[0], safe_indices[1]):
            x = clearance_data['x_coords'][i]
            y = clearance_data['y_coords'][j]
            waypoints.append([x, y, height])
        
        return np.array(waypoints)


def main():
    """Main function for analysis utilities"""
    if len(sys.argv) < 2:
        print("Usage: python3 pointcloud_utils.py <command> [options]")
        print("Commands:")
        print("  analyze <pointcloud_file>  - Analyze pointcloud statistics")
        print("  clearance <pointcloud_file> <height> - Generate clearance map")
        print("  waypoints <pointcloud_file> <height> <clearance> - Generate waypoints")
        return
    
    command = sys.argv[1]
    
    if command == "analyze" and len(sys.argv) >= 3:
        pointcloud_file = sys.argv[2]
        analyzer = PointcloudAnalyzer(pointcloud_file)
        
        print("\n=== Pointcloud Analysis ===")
        bounds = analyzer.get_bounds()
        print(f"Bounds: {bounds}")
        
        density = analyzer.analyze_density(voxel_size=1.0)
        print(f"\nDensity Analysis (1m voxels):")
        print(f"  Occupied voxels: {density['occupied_voxels']}/{density['total_voxels']}")
        print(f"  Occupancy ratio: {density['occupied_voxels']/density['total_voxels']:.2%}")
        print(f"  Max density: {density['max_density']:.1f} points/voxel")
        print(f"  Mean density: {density['mean_density']:.1f} points/voxel")
    
    elif command == "clearance" and len(sys.argv) >= 4:
        pointcloud_file = sys.argv[2]
        height = float(sys.argv[3])
        
        analyzer = PointcloudAnalyzer(pointcloud_file)
        clearance_data = analyzer.find_clearance_map(height)
        
        output_path = pointcloud_file.replace('.txt', f'_clearance_{height}m.png')
        analyzer.plot_clearance_map(clearance_data, output_path)
    
    elif command == "waypoints" and len(sys.argv) >= 5:
        pointcloud_file = sys.argv[2]
        height = float(sys.argv[3])
        min_clearance = float(sys.argv[4])
        
        analyzer = PointcloudAnalyzer(pointcloud_file)
        waypoints = analyzer.generate_navigation_waypoints(height, min_clearance)
        
        output_path = pointcloud_file.replace('.txt', f'_waypoints_{height}m.txt')
        np.savetxt(output_path, waypoints, fmt='%.2f', header='X Y Z')
        print(f"Generated {len(waypoints)} waypoints, saved to: {output_path}")
    
    else:
        print("Invalid command or missing arguments")


if __name__ == "__main__":
    main()