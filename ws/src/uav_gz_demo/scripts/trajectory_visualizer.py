#!/usr/bin/env python3
"""
Trajectory Visualizer for Gazebo Simulation
Displays the planned trajectory as visual markers in Gazebo and RViz
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA, Header
import csv
import numpy as np
import os
from scipy.spatial.transform import Rotation


class TrajectoryVisualizer(Node):
    """Visualize trajectory in Gazebo and RViz"""
    
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('marker_scale', 0.2)
        self.declare_parameter('line_width', 0.1)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('trajectory_color', [1.0, 0.0, 0.0, 0.8])  # Red with alpha
        self.declare_parameter('current_point_color', [0.0, 1.0, 0.0, 1.0])  # Green
        self.declare_parameter('show_orientation', True)
        self.declare_parameter('show_waypoints', True)
        # self.declare_parameter('use_sim_time', True)
        
        # Get parameters
        self.csv_path = self.get_parameter('csv_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.line_width = self.get_parameter('line_width').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.trajectory_color = self.get_parameter('trajectory_color').value
        self.current_point_color = self.get_parameter('current_point_color').value
        self.show_orientation = self.get_parameter('show_orientation').value
        self.show_waypoints = self.get_parameter('show_waypoints').value
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        # Load trajectory
        self.trajectory_points = []
        self.load_trajectory()
        
        # Create timer for publishing markers
        timer_period = 1.0 / self.publish_rate
        self.marker_timer = self.create_timer(timer_period, self.publish_markers)
        
        # Track current trajectory point (for highlighting current target)
        self.current_trajectory_index = 0
        
        self.get_logger().info(f"Trajectory Visualizer initialized")
        self.get_logger().info(f"Loaded {len(self.trajectory_points)} trajectory points from {self.csv_path}")
        self.get_logger().info(f"Publishing markers to /trajectory_markers in frame '{self.frame_id}'")
    
    def load_trajectory(self):
        """Load trajectory from CSV file"""
        if not self.csv_path or not os.path.exists(self.csv_path):
            self.get_logger().error(f"Trajectory CSV file not found: {self.csv_path}")
            return
        
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)  # Skip header if present
                
                for row in reader:
                    if len(row) >= 4:  # At least time, x, y, z
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            z = float(row[2])
                            
                            # Optional yaw, roll, pitch
                            yaw = float(row[3]) if len(row) > 4 else 0.0
                            roll = float(row[4]) if len(row) > 5 else 0.0
                            pitch = float(row[5]) if len(row) > 6 else 0.0
                            
                            self.trajectory_points.append({
                                'position': [x, y, z],
                                'orientation': [roll, pitch, yaw]  # RPY format
                            })
                        except (ValueError, IndexError) as e:
                            self.get_logger().warn(f"Skipping invalid trajectory row: {row} ({e})")
                            continue
            
            self.get_logger().info(f"Successfully loaded {len(self.trajectory_points)} trajectory points")
            
        except Exception as e:
            self.get_logger().error(f"Error loading trajectory: {e}")
    
    def create_trajectory_line_marker(self) -> Marker:
        """Create line strip marker for the full trajectory"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "trajectory_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set scale (line width)
        marker.scale = Vector3()
        marker.scale.x = self.line_width
        
        # Set color
        marker.color = ColorRGBA()
        marker.color.r = self.trajectory_color[0]
        marker.color.g = self.trajectory_color[1]
        marker.color.b = self.trajectory_color[2]
        marker.color.a = self.trajectory_color[3]
        
        # Add all trajectory points
        for point_data in self.trajectory_points:
            point = Point()
            point.x = point_data['position'][0]
            point.y = point_data['position'][1]
            point.z = point_data['position'][2]
            marker.points.append(point)
        
        marker.lifetime.sec = 0  # Persistent
        
        return marker
    
    def create_waypoint_markers(self) -> list:
        """Create sphere markers for individual waypoints"""
        markers = []
        
        for i, point_data in enumerate(self.trajectory_points):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "trajectory_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose = Pose()
            marker.pose.position.x = point_data['position'][0]
            marker.pose.position.y = point_data['position'][1]
            marker.pose.position.z = point_data['position'][2]
            marker.pose.orientation.w = 1.0  # No rotation for spheres
            
            # Set scale
            marker.scale = Vector3()
            marker.scale.x = self.marker_scale * 0.5
            marker.scale.y = self.marker_scale * 0.5
            marker.scale.z = self.marker_scale * 0.5
            
            # Set color (slightly transparent)
            marker.color = ColorRGBA()
            marker.color.r = self.trajectory_color[0]
            marker.color.g = self.trajectory_color[1]
            marker.color.b = self.trajectory_color[2]
            marker.color.a = 0.6
            
            marker.lifetime.sec = 0  # Persistent
            
            markers.append(marker)
        
        return markers
    
    def create_orientation_markers(self) -> list:
        """Create arrow markers showing orientation at each waypoint"""
        markers = []
        
        for i, point_data in enumerate(self.trajectory_points):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            
            marker.ns = "trajectory_orientations"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set position
            marker.pose = Pose()
            marker.pose.position.x = point_data['position'][0]
            marker.pose.position.y = point_data['position'][1]
            marker.pose.position.z = point_data['position'][2]
            
            # Convert RPY to quaternion
            roll, pitch, yaw = point_data['orientation']
            r = Rotation.from_euler('xyz', [roll, pitch, yaw])
            quat = r.as_quat()  # Returns [x, y, z, w]
            
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            
            # Set scale (arrow size)
            marker.scale = Vector3()
            marker.scale.x = self.marker_scale * 2.0  # Length
            marker.scale.y = self.marker_scale * 0.2  # Width
            marker.scale.z = self.marker_scale * 0.2  # Height
            
            # Set color (blue for orientation)
            marker.color = ColorRGBA()
            marker.color.r = 0.0
            marker.color.g = 0.0  
            marker.color.b = 1.0
            marker.color.a = 0.7
            
            marker.lifetime.sec = 0  # Persistent
            
            markers.append(marker)
        
        return markers
    
    def create_start_end_markers(self) -> list:
        """Create special markers for start and end points"""
        markers = []
        
        if not self.trajectory_points:
            return markers
        
        # Start marker (green)
        start_marker = Marker()
        start_marker.header = Header()
        start_marker.header.frame_id = self.frame_id
        start_marker.header.stamp = self.get_clock().now().to_msg()
        
        start_marker.ns = "trajectory_start_end"
        start_marker.id = 0
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        
        start_point = self.trajectory_points[0]
        start_marker.pose = Pose()
        start_marker.pose.position.x = start_point['position'][0]
        start_marker.pose.position.y = start_point['position'][1]
        start_marker.pose.position.z = start_point['position'][2]
        start_marker.pose.orientation.w = 1.0
        
        start_marker.scale = Vector3()
        start_marker.scale.x = self.marker_scale * 1.5
        start_marker.scale.y = self.marker_scale * 1.5
        start_marker.scale.z = self.marker_scale * 0.2
        
        start_marker.color = ColorRGBA()
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 0.8
        
        start_marker.lifetime.sec = 0
        markers.append(start_marker)
        
        # End marker (red)
        end_marker = Marker()
        end_marker.header = Header()
        end_marker.header.frame_id = self.frame_id
        end_marker.header.stamp = self.get_clock().now().to_msg()
        
        end_marker.ns = "trajectory_start_end"
        end_marker.id = 1
        end_marker.type = Marker.CYLINDER
        end_marker.action = Marker.ADD
        
        end_point = self.trajectory_points[-1]
        end_marker.pose = Pose()
        end_marker.pose.position.x = end_point['position'][0]
        end_marker.pose.position.y = end_point['position'][1]
        end_marker.pose.position.z = end_point['position'][2]
        end_marker.pose.orientation.w = 1.0
        
        end_marker.scale = Vector3()
        end_marker.scale.x = self.marker_scale * 1.5
        end_marker.scale.y = self.marker_scale * 1.5
        end_marker.scale.z = self.marker_scale * 0.2
        
        end_marker.color = ColorRGBA()
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        end_marker.color.a = 0.8
        
        end_marker.lifetime.sec = 0
        markers.append(end_marker)
        
        return markers
    
    def publish_markers(self):
        """Publish all trajectory markers"""
        if not self.trajectory_points:
            return
        
        marker_array = MarkerArray()
        
        # Add trajectory line
        line_marker = self.create_trajectory_line_marker()
        marker_array.markers.append(line_marker)
        
        # Add waypoint markers if enabled
        if self.show_waypoints:
            waypoint_markers = self.create_waypoint_markers()
            marker_array.markers.extend(waypoint_markers)
        
        # Add orientation markers if enabled
        if self.show_orientation:
            orientation_markers = self.create_orientation_markers()
            marker_array.markers.extend(orientation_markers)
        
        # Add start/end markers
        start_end_markers = self.create_start_end_markers()
        marker_array.markers.extend(start_end_markers)
        
        self.get_logger().info("Publishing trajectory markers")
        # Publish marker array
        self.marker_pub.publish(marker_array)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        visualizer = TrajectoryVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()