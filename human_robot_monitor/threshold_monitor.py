#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Int32
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.time import Time

class ThresholdMonitorNode(Node):
    def __init__(self):
        super().__init__('threshold_monitor_node')
        
        # Initialize position variables
        self.lidar_positions = []
        self.rgbd_positions = []
        
        # Define RGBD camera FOV parameters
        self.camera_distance = 3.5  # Distance from camera to far plane
        self.fov_width = 3.0       # Width of FOV at far plane (1.5m each side)
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('position_threshold', 0.5),  # Maximum allowed distance between corresponding detections
                ('matching_timeout', 0.1),    # Time window to consider detections as simultaneous
            ]
        )
        
        # Create subscribers
        self.lidar_sub = self.create_subscription(
            Point, 
            '/lidar_human_position_sensor',
            self.lidar_callback,
            10)
            
        self.rgbd_sub = self.create_subscription(
            Point,
            '/rgbd_human_position_sensor',
            self.rgbd_callback,
            10)
            
        # Create publisher for consistency status
        self.status_pub = self.create_publisher(Int32, 'detection_consistency', 10)
        
        # Timer for periodic consistency check
        self.timer = self.create_timer(0.05, self.check_consistency)  # 20Hz check
        
        self.get_logger().info('Threshold monitor node has started')

    def transform_lidar_to_kinect(self, point):
        """Transform a point from LIDAR frame to Kinect frame"""
        try:
            # Create PointStamped message
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'laser'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = point
            
            # Transform to Kinect frame
            transformed = self.tf_buffer.transform(point_stamped, 'kinect_link')
            return transformed.point
        except Exception as e:
            self.get_logger().error(f'Failed to transform point: {str(e)}')
            return None

    def is_point_in_fov(self, x, y):
        """Check if a point is within the RGBD camera's FOV"""
        # In sensor frame, FOV is a triangle:
        # - Origin at (0,0,0)
        # - Extends along positive Y axis
        # - Width increases with distance
        if y <= 0:  # Point is behind the camera
            return False
            
        # Calculate the allowed width at this y-distance
        allowed_width = (y / self.camera_distance) * (self.fov_width / 2)
        
        # Check if point is within the FOV triangle
        return abs(x) <= allowed_width and y <= self.camera_distance

    def lidar_callback(self, msg):
        """Store LIDAR detection if it's within RGBD FOV"""
        # Transform LIDAR position to Kinect frame
        transformed_point = self.transform_lidar_to_kinect(msg)
        if transformed_point is None:
            return
            
        # Check if transformed point is in FOV
        if self.is_point_in_fov(transformed_point.x, transformed_point.y):
            self.lidar_positions.append((transformed_point.x, transformed_point.y))
        
        # Keep only the most recent detection(s)
        if len(self.lidar_positions) > 5:
            self.lidar_positions.pop(0)

    def rgbd_callback(self, msg):
        """Store RGBD detection"""
        self.rgbd_positions.append((msg.x, msg.y))
        
        # Keep only the most recent detection(s)
        if len(self.rgbd_positions) > 5:
            self.rgbd_positions.pop(0)

    def check_consistency(self):
        """Check if LIDAR and RGBD detections are consistent"""
        # Get parameters
        threshold = self.get_parameter('position_threshold').value
        
        # Clear old detections
        self.lidar_positions = self.lidar_positions[-5:] if self.lidar_positions else []
        self.rgbd_positions = self.rgbd_positions[-5:] if self.rgbd_positions else []
        
        # If no detections in FOV, consider it consistent
        if not self.lidar_positions or not self.rgbd_positions:
            status_msg = Int32()
            status_msg.data = 1
            self.status_pub.publish(status_msg)
            return
            
        # For each LIDAR detection, find the closest RGBD detection
        all_matched = True
        used_rgbd_indices = set()
        
        for i, lidar_pos in enumerate(self.lidar_positions):
            min_dist = float('inf')
            best_match = None
            best_idx = None
            
            # Find the closest unused RGBD detection
            for j, rgbd_pos in enumerate(self.rgbd_positions):
                if j in used_rgbd_indices:
                    continue
                    
                dist = math.sqrt(
                    (lidar_pos[0] - rgbd_pos[0])**2 + 
                    (lidar_pos[1] - rgbd_pos[1])**2
                )
                
                if dist < min_dist:
                    min_dist = dist
                    best_match = rgbd_pos
                    best_idx = j
            
            # Check if the match is within threshold
            if min_dist <= threshold and best_idx is not None:
                used_rgbd_indices.add(best_idx)
            else:
                all_matched = False
                break
        
        # Check if all RGBD detections were matched
        if len(self.rgbd_positions) > len(used_rgbd_indices):
            all_matched = False
        
        # Publish status
        status_msg = Int32()
        status_msg.data = 1 if all_matched else 0
        self.status_pub.publish(status_msg)
        
        # Log for debugging
        self.get_logger().debug(
            f'LIDAR detections: {len(self.lidar_positions)}, '
            f'RGBD detections: {len(self.rgbd_positions)}, '
            f'Consistent: {all_matched}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 