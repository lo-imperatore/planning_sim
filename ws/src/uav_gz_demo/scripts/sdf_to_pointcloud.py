#!/usr/bin/env python3
"""
SDF to Pointcloud Generator

This script parses an SDF (Simulation Description Format) file and generates
a pointcloud representation of all the geometric objects in the world.
Supports boxes, cylinders, and other basic geometric primitives.
"""

import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import os
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from scipy.spatial.transform import Rotation


@dataclass
class Pose:
    """Represents a 6DOF pose with position and orientation"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    @classmethod
    def from_string(cls, pose_str: str) -> 'Pose':
        """Parse pose from SDF string format: 'x y z roll pitch yaw'"""
        if not pose_str.strip():
            return cls()
        values = [float(x) for x in pose_str.strip().split()]
        if len(values) == 6:
            return cls(*values)
        elif len(values) == 3:
            return cls(values[0], values[1], values[2])
        else:
            return cls()
    
    def to_transform_matrix(self) -> np.ndarray:
        """Convert pose to 4x4 transformation matrix"""
        # Create rotation matrix from roll, pitch, yaw (XYZ Euler angles)
        r = Rotation.from_euler('xyz', [self.roll, self.pitch, self.yaw])
        rotation_matrix = r.as_matrix()
        
        # Create 4x4 transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = [self.x, self.y, self.z]
        
        return transform


@dataclass
class GeometryInfo:
    """Information about a geometric object"""
    geometry_type: str
    pose: Pose
    parameters: Dict
    model_name: str
    link_name: str


class SDFParser:
    """Parser for SDF files to extract geometric information"""
    
    def __init__(self, sdf_file_path: str):
        self.sdf_file_path = sdf_file_path
        self.geometries: List[GeometryInfo] = []
    
    def parse(self) -> List[GeometryInfo]:
        """Parse the SDF file and extract all geometric objects"""
        tree = ET.parse(self.sdf_file_path)
        root = tree.getroot()
        
        # Find the world element
        world = root.find('world')
        if world is None:
            raise ValueError("No world element found in SDF file")
        
        # Parse all models in the world
        for model in world.findall('model'):
            model_name = model.get('name', 'unnamed_model')
            self._parse_model(model, model_name)
        
        return self.geometries
    
    def _parse_model(self, model_element: ET.Element, model_name: str):
        """Parse a model element and extract its geometries"""
        # Get model pose
        model_pose_elem = model_element.find('pose')
        model_pose = Pose.from_string(model_pose_elem.text if model_pose_elem is not None else "")
        
        # Parse all links in the model
        for link in model_element.findall('link'):
            link_name = link.get('name', 'unnamed_link')
            self._parse_link(link, model_name, link_name, model_pose)
    
    def _parse_link(self, link_element: ET.Element, model_name: str, link_name: str, model_pose: Pose):
        """Parse a link element and extract its collision geometries"""
        # Get link pose
        link_pose_elem = link_element.find('pose')
        link_pose = Pose.from_string(link_pose_elem.text if link_pose_elem is not None else "")
        
        # Combine model and link poses
        combined_pose = self._combine_poses(model_pose, link_pose)
        
        # Parse collision geometries
        for collision in link_element.findall('collision'):
            geometry_elem = collision.find('geometry')
            if geometry_elem is not None:
                self._parse_geometry(geometry_elem, combined_pose, model_name, link_name)
    
    def _combine_poses(self, parent_pose: Pose, child_pose: Pose) -> Pose:
        """Combine two poses using transformation matrices"""
        parent_transform = parent_pose.to_transform_matrix()
        child_transform = child_pose.to_transform_matrix()
        
        # Combine transformations
        combined_transform = parent_transform @ child_transform
        
        # Extract position
        x, y, z = combined_transform[:3, 3]
        
        # Extract rotation (convert back to Euler angles)
        rotation_matrix = combined_transform[:3, :3]
        r = Rotation.from_matrix(rotation_matrix)
        roll, pitch, yaw = r.as_euler('xyz')
        
        return Pose(x, y, z, roll, pitch, yaw)
    
    def _parse_geometry(self, geometry_elem: ET.Element, pose: Pose, model_name: str, link_name: str):
        """Parse a geometry element and create GeometryInfo"""
        for geom_type in geometry_elem:
            if geom_type.tag == 'box':
                size_elem = geom_type.find('size')
                if size_elem is not None:
                    size = [float(x) for x in size_elem.text.split()]
                    parameters = {'size': size}
                    self.geometries.append(GeometryInfo('box', pose, parameters, model_name, link_name))
            
            elif geom_type.tag == 'cylinder':
                radius_elem = geom_type.find('radius')
                length_elem = geom_type.find('length')
                if radius_elem is not None and length_elem is not None:
                    parameters = {
                        'radius': float(radius_elem.text),
                        'length': float(length_elem.text)
                    }
                    self.geometries.append(GeometryInfo('cylinder', pose, parameters, model_name, link_name))
            
            elif geom_type.tag == 'sphere':
                radius_elem = geom_type.find('radius')
                if radius_elem is not None:
                    parameters = {'radius': float(radius_elem.text)}
                    self.geometries.append(GeometryInfo('sphere', pose, parameters, model_name, link_name))


class PointcloudGenerator:
    """Generate pointclouds from geometric primitives"""
    
    def __init__(self, point_density: float = 0.1):
        """
        Initialize the pointcloud generator
        
        Args:
            point_density: Distance between points in meters (smaller = denser)
        """
        self.point_density = point_density
    
    def generate_pointcloud(self, geometries: List[GeometryInfo]) -> np.ndarray:
        """Generate a pointcloud from a list of geometries"""
        all_points = []
        
        for geom in geometries:
            if geom.geometry_type == 'box':
                points = self._generate_box_pointcloud(geom)
            elif geom.geometry_type == 'cylinder':
                points = self._generate_cylinder_pointcloud(geom)
            elif geom.geometry_type == 'sphere':
                points = self._generate_sphere_pointcloud(geom)
            else:
                print(f"Warning: Geometry type '{geom.geometry_type}' not supported")
                continue
            
            if len(points) > 0:
                all_points.append(points)
        
        if all_points:
            return np.vstack(all_points)
        else:
            return np.empty((0, 3))
    
    def _generate_box_pointcloud(self, geom: GeometryInfo) -> np.ndarray:
        """Generate pointcloud for a box geometry"""
        size = geom.parameters['size']
        sx, sy, sz = size[0], size[1], size[2]
        
        # Generate points on all 6 faces of the box
        points = []
        
        # Calculate number of points per dimension
        nx = max(2, int(sx / self.point_density))
        ny = max(2, int(sy / self.point_density))
        nz = max(2, int(sz / self.point_density))
        
        # Face 1: +X face (x = sx/2)
        y_vals = np.linspace(-sy/2, sy/2, ny)
        z_vals = np.linspace(-sz/2, sz/2, nz)
        for y in y_vals:
            for z in z_vals:
                points.append([sx/2, y, z])
        
        # Face 2: -X face (x = -sx/2)
        for y in y_vals:
            for z in z_vals:
                points.append([-sx/2, y, z])
        
        # Face 3: +Y face (y = sy/2)
        x_vals = np.linspace(-sx/2, sx/2, nx)
        for x in x_vals:
            for z in z_vals:
                points.append([x, sy/2, z])
        
        # Face 4: -Y face (y = -sy/2)
        for x in x_vals:
            for z in z_vals:
                points.append([x, -sy/2, z])
        
        # Face 5: +Z face (z = sz/2)
        for x in x_vals:
            for y in y_vals:
                points.append([x, y, sz/2])
        
        # Face 6: -Z face (z = -sz/2)
        for x in x_vals:
            for y in y_vals:
                points.append([x, y, -sz/2])
        
        points = np.array(points)
        
        # Apply transformation
        return self._transform_points(points, geom.pose)
    
    def _generate_cylinder_pointcloud(self, geom: GeometryInfo) -> np.ndarray:
        """Generate pointcloud for a cylinder geometry"""
        radius = geom.parameters['radius']
        length = geom.parameters['length']
        
        points = []
        
        # Calculate number of points
        n_circumference = max(8, int(2 * np.pi * radius / self.point_density))
        n_length = max(2, int(length / self.point_density))
        
        # Generate points on curved surface
        theta_vals = np.linspace(0, 2*np.pi, n_circumference)
        z_vals = np.linspace(-length/2, length/2, n_length)
        
        for theta in theta_vals:
            for z in z_vals:
                x = radius * np.cos(theta)
                y = radius * np.sin(theta)
                points.append([x, y, z])
        
        # Generate points on top and bottom caps
        n_radial = max(2, int(radius / self.point_density))
        r_vals = np.linspace(0, radius, n_radial)
        
        for r in r_vals:
            n_theta = max(1, int(2 * np.pi * r / self.point_density)) if r > 0 else 1
            theta_vals_cap = np.linspace(0, 2*np.pi, n_theta)
            
            for theta in theta_vals_cap:
                x = r * np.cos(theta) if r > 0 else 0
                y = r * np.sin(theta) if r > 0 else 0
                
                # Top cap
                points.append([x, y, length/2])
                # Bottom cap
                points.append([x, y, -length/2])
        
        points = np.array(points)
        
        # Apply transformation
        return self._transform_points(points, geom.pose)
    
    def _generate_sphere_pointcloud(self, geom: GeometryInfo) -> np.ndarray:
        """Generate pointcloud for a sphere geometry"""
        radius = geom.parameters['radius']
        
        points = []
        
        # Calculate number of points
        n_phi = max(4, int(np.pi * radius / self.point_density))  # latitude
        n_theta = max(8, int(2 * np.pi * radius / self.point_density))  # longitude
        
        phi_vals = np.linspace(0, np.pi, n_phi)
        theta_vals = np.linspace(0, 2*np.pi, n_theta)
        
        for phi in phi_vals:
            for theta in theta_vals:
                x = radius * np.sin(phi) * np.cos(theta)
                y = radius * np.sin(phi) * np.sin(theta)
                z = radius * np.cos(phi)
                points.append([x, y, z])
        
        points = np.array(points)
        
        # Apply transformation
        return self._transform_points(points, geom.pose)
    
    def _transform_points(self, points: np.ndarray, pose: Pose) -> np.ndarray:
        """Apply pose transformation to points"""
        if len(points) == 0:
            return points
        
        # Convert to homogeneous coordinates
        homogeneous_points = np.hstack([points, np.ones((points.shape[0], 1))])
        
        # Apply transformation
        transform = pose.to_transform_matrix()
        transformed = (transform @ homogeneous_points.T).T
        
        # Return 3D points
        return transformed[:, :3]


class PointcloudVisualizer:
    """Visualize pointclouds using matplotlib"""
    
    @staticmethod
    def plot_pointcloud(points: np.ndarray, title: str = "Pointcloud", 
                       point_size: float = 1.0, save_path: Optional[str] = None):
        """Plot pointcloud in 3D"""
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        if len(points) > 0:
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                      s=point_size, alpha=0.6, c='blue')
            
            # Set equal aspect ratio
            max_range = np.array([points[:, 0].max() - points[:, 0].min(),
                                 points[:, 1].max() - points[:, 1].min(),
                                 points[:, 2].max() - points[:, 2].min()]).max() / 2.0
            
            mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
            mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
            mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
            
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(title)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Pointcloud visualization saved to: {save_path}")
        
        plt.show()
    
    @staticmethod
    def save_pointcloud_txt(points: np.ndarray, filepath: str):
        """Save pointcloud to text file (X Y Z format)"""
        np.savetxt(filepath, points, fmt='%.6f', header='X Y Z', comments='')
        print(f"Pointcloud saved to: {filepath}")
    
    @staticmethod
    def save_pointcloud_ply(points: np.ndarray, filepath: str):
        """Save pointcloud to PLY format"""
        header = f"""ply
format ascii 1.0
element vertex {len(points)}
property float x
property float y
property float z
end_header
"""
        
        with open(filepath, 'w') as f:
            f.write(header)
            for point in points:
                f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
        
        print(f"Pointcloud saved to PLY format: {filepath}")


def main():
    """Main function to parse arguments and generate pointcloud"""
    parser = argparse.ArgumentParser(description='Generate pointcloud from SDF file')
    parser.add_argument('sdf_file', help='Path to the SDF file')
    parser.add_argument('--density', type=float, default=0.2, 
                       help='Point density (distance between points in meters)')
    parser.add_argument('--output', type=str, help='Output file path (without extension)')
    parser.add_argument('--format', choices=['txt', 'ply', 'both'], default='both',
                       help='Output format')
    parser.add_argument('--visualize', action='store_true', 
                       help='Display 3D visualization')
    parser.add_argument('--point-size', type=float, default=1.0,
                       help='Point size for visualization')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.sdf_file):
        print(f"Error: SDF file '{args.sdf_file}' not found")
        return
    
    print(f"Parsing SDF file: {args.sdf_file}")
    
    # Parse SDF file
    parser_obj = SDFParser(args.sdf_file)
    geometries = parser_obj.parse()
    
    print(f"Found {len(geometries)} geometric objects")
    
    # Generate pointcloud
    generator = PointcloudGenerator(point_density=args.density)
    pointcloud = generator.generate_pointcloud(geometries)
    
    print(f"Generated pointcloud with {len(pointcloud)} points")
    
    # Save pointcloud
    if args.output:
        base_path = args.output
    else:
        base_path = os.path.splitext(args.sdf_file)[0] + '_pointcloud'
    
    if args.format in ['txt', 'both']:
        PointcloudVisualizer.save_pointcloud_txt(pointcloud, base_path + '.txt')
    
    if args.format in ['ply', 'both']:
        PointcloudVisualizer.save_pointcloud_ply(pointcloud, base_path + '.ply')
    
    # Visualize if requested
    if args.visualize:
        title = f"Pointcloud from {os.path.basename(args.sdf_file)}"
        save_path = base_path + '_visualization.png' if args.output else None
        PointcloudVisualizer.plot_pointcloud(pointcloud, title, args.point_size, save_path)


if __name__ == "__main__":
    main()