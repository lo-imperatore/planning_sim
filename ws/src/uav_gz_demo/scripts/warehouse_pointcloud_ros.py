#!/usr/bin/env python3
"""
ROS integration script for warehouse pointcloud
Publishes the generated pointcloud to ROS topics for navigation planning
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import sys
import os

# Add the scripts directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from sdf_to_pointcloud import SDFParser, PointcloudGenerator


class WarehousePointcloudPublisher:
    """ROS node to publish warehouse pointcloud"""
    
    def __init__(self):
        rospy.init_node('warehouse_pointcloud_publisher', anonymous=True)
        
        # Publishers
        self.obstacle_pub = rospy.Publisher('/warehouse/obstacles', PointCloud2, queue_size=1, latch=True)
        self.full_env_pub = rospy.Publisher('/warehouse/environment', PointCloud2, queue_size=1, latch=True)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.point_density = rospy.get_param('~point_density', 0.15)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        rospy.loginfo(f"Warehouse Pointcloud Publisher initialized")
        rospy.loginfo(f"Frame ID: {self.frame_id}")
        rospy.loginfo(f"Point density: {self.point_density}m")
        rospy.loginfo(f"Publish rate: {self.publish_rate}Hz")
    
    def generate_pointcloud(self):
        """Generate pointcloud from warehouse SDF"""
        warehouse_sdf = "/home/pietro/planning_sim/ws/src/uav_gz_demo/worlds/x3_warehouse_challenging.sdf"
        
        if not os.path.exists(warehouse_sdf):
            rospy.logerr(f"Warehouse SDF file not found: {warehouse_sdf}")
            return None
        
        rospy.loginfo("Parsing warehouse SDF...")
        parser = SDFParser(warehouse_sdf)
        geometries = parser.parse()
        
        rospy.loginfo(f"Found {len(geometries)} geometric objects")
        
        rospy.loginfo("Generating pointcloud...")
        generator = PointcloudGenerator(point_density=self.point_density)
        points = generator.generate_pointcloud(geometries)
        
        rospy.loginfo(f"Generated {len(points)} points")
        
        return points
    
    def points_to_pointcloud2(self, points, timestamp=None):
        """Convert numpy array to PointCloud2 message"""
        if timestamp is None:
            timestamp = rospy.Time.now()
        
        header = Header()
        header.stamp = timestamp
        header.frame_id = self.frame_id
        
        # Define PointCloud2 fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        # Pack point data
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = b''.join(cloud_data)
        cloud_msg.is_dense = True
        
        return cloud_msg
    
    def filter_height_range(self, points, z_min=0.1, z_max=5.5):
        """Filter points within drone flying height range"""
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        return points[mask]
    
    def run(self):
        """Main execution loop"""
        # Generate pointcloud once
        points = self.generate_pointcloud()
        
        if points is None or len(points) == 0:
            rospy.logerr("Failed to generate pointcloud")
            return
        
        # Create different filtered versions
        obstacle_points = self.filter_height_range(points, 0.1, 5.5)  # Drone flying range
        
        rospy.loginfo(f"Obstacle points in flying range: {len(obstacle_points)}")
        
        # Convert to ROS messages
        full_cloud_msg = self.points_to_pointcloud2(points)
        obstacle_cloud_msg = self.points_to_pointcloud2(obstacle_points)
        
        # Publishing loop
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo("Publishing pointclouds...")
        
        while not rospy.is_shutdown():
            # Update timestamps
            current_time = rospy.Time.now()
            full_cloud_msg.header.stamp = current_time
            obstacle_cloud_msg.header.stamp = current_time
            
            # Publish
            self.full_env_pub.publish(full_cloud_msg)
            self.obstacle_pub.publish(obstacle_cloud_msg)
            
            # Log info periodically
            if rospy.get_time() % 10 < 1.0/self.publish_rate:  # Every 10 seconds
                rospy.loginfo(f"Publishing pointclouds - Full: {len(points)}, Obstacles: {len(obstacle_points)} points")
            
            rate.sleep()


def main():
    try:
        publisher = WarehousePointcloudPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Warehouse pointcloud publisher shutting down")
    except Exception as e:
        rospy.logerr(f"Error in warehouse pointcloud publisher: {e}")


if __name__ == '__main__':
    main()