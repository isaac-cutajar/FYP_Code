#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int32
import math
import numpy as np
from collections import deque
import time

class SpeedScalerNode(Node):
    def __init__(self):
        super().__init__('speed_scaler_node')
        
        # Initialize position variables with timestamps
        self.lidar_position = None
        self.rgbd_position = None
        self.lidar_position_time = None
        self.rgbd_position_time = None
        
        # Data timeout in seconds
        self.data_timeout = 0.5  # 500ms timeout
        
        # Define distance thresholds and speed scales
        self.min_distance = 0.5  # meters
        self.max_distance = 2.0  # meters
        self.min_speed = 0.1     # 10% speed
        self.max_speed = 1.0     # 100% speed
        
        # Buffer to store recent distance values for averaging
        self.distance_buffer = deque(maxlen=10)
        
        # Create subscribers
        self.lidar_sub = self.create_subscription(
            Point, 
            '/lidar_human_position',
            self.lidar_callback,
            10)
            
        self.rgbd_sub = self.create_subscription(
            Point,
            '/rgbd_human_position',
            self.rgbd_callback,
            10)
            
        # Create publishers
        self.speed_pub = self.create_publisher(
            Float64, 
            '/speed_scale_fraction',
            10)
            
        self.detection_pub = self.create_publisher(
            Int32,
            'human_detected',
            10)
            
        # Timer for periodic speed update
        self.timer = self.create_timer(0.05, self.update_speed)  # 20Hz update
        
        self.get_logger().info('Speed scaler node has started')

    def get_distance(self, point):
        """Calculate Euclidean distance from point to origin"""
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def is_data_valid(self, data, timestamp):
        """Check if data is still valid based on timestamp"""
        if data is None or timestamp is None:
            return False
        return (time.time() - timestamp) < self.data_timeout

    def calculate_speed_scale(self, distance):
        """Calculate speed scale based on distance"""
        if distance <= self.min_distance:
            return self.min_speed
        elif distance >= self.max_distance:
            return self.max_speed
        else:
            # Linear interpolation between min and max speed
            distance_ratio = (distance - self.min_distance) / (self.max_distance - self.min_distance)
            return self.min_speed + distance_ratio * (self.max_speed - self.min_speed)

    def lidar_callback(self, msg):
        """Process LIDAR position data"""
        self.lidar_position = msg
        self.lidar_position_time = time.time()

    def rgbd_callback(self, msg):
        """Process RGBD position data"""
        self.rgbd_position = msg
        self.rgbd_position_time = time.time()

    def update_speed(self):
        """Update robot speed based on closest human distance"""
        current_time = time.time()
        
        # Check if data is too old and clear if necessary
        if not self.is_data_valid(self.lidar_position, self.lidar_position_time):
            self.lidar_position = None
            
        if not self.is_data_valid(self.rgbd_position, self.rgbd_position_time):
            self.rgbd_position = None
            
        distances = []
        
        # Calculate distances from both sensors if available
        if self.lidar_position is not None:
            distances.append(self.get_distance(self.lidar_position))
            
        if self.rgbd_position is not None:
            distances.append(self.get_distance(self.rgbd_position))
            
        # If no valid distances, use maximum speed and publish no detection
        if not distances:
            speed_scale = self.max_speed
            detection_status = 0
        else:
            # Get closest distance and calculate speed scale
            min_distance = min(distances)
            speed_scale = self.calculate_speed_scale(min_distance)
            detection_status = 1
        
        # Publish speed scale
        speed_msg = Float64()
        speed_msg.data = float(speed_scale)
        self.speed_pub.publish(speed_msg)
        
        # Publish detection status
        status_msg = Int32()
        status_msg.data = detection_status
        self.detection_pub.publish(status_msg)
        
        # Log for debugging
        self.get_logger().debug(
            f'Closest distance: {min(distances) if distances else "N/A"}m, '
            f'Speed scale: {speed_scale:.2f}, '
            f'Human detected: {detection_status}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SpeedScalerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
