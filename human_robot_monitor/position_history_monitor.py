#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import math
import time
from collections import deque

class PositionHistoryMonitorNode(Node):
    def __init__(self):
        super().__init__('position_history_monitor_node')
        
        # Initialize position history
        self.history_duration = 0.5  # seconds to keep in history
        self.distance_threshold = 2.0  # meters
        
        # Store tuples of (distance, timestamp)
        self.position_history = deque()
        
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
            
        # Create publisher for detection status
        self.status_pub = self.create_publisher(
            Int32,
            'human_history_status',
            10)
            
        # Timer for periodic history check
        self.timer = self.create_timer(0.05, self.check_history)  # 20Hz check
        
        self.get_logger().info('Position history monitor node has started')

    def get_distance(self, point):
        """Calculate Euclidean distance from point to origin"""
        return math.sqrt(point.x ** 2 + point.y ** 2)

    def update_history(self, distance):
        """Add new distance to history and remove old entries"""
        current_time = time.time()
        
        # Add new distance
        self.position_history.append((distance, current_time))
        
        # Remove entries older than history_duration
        while self.position_history and \
              (current_time - self.position_history[0][1]) > self.history_duration:
            self.position_history.popleft()

    def was_human_detected(self):
        """Check if any distance in history was below threshold"""
        return any(distance < self.distance_threshold for distance, _ in self.position_history)

    def lidar_callback(self, msg):
        """Process LIDAR position data"""
        distance = self.get_distance(msg)
        self.update_history(distance)

    def rgbd_callback(self, msg):
        """Process RGBD position data"""
        distance = self.get_distance(msg)
        self.update_history(distance)

    def check_history(self):
        """Check position history and publish status"""
        if not self.position_history:
            # No recent detections
            status = 0
        else:
            # Check if any recent detection was within threshold
            status = 1 if self.was_human_detected() else 0
        
        # Publish status
        status_msg = Int32()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        # Log for debugging
        if self.position_history:
            latest_distance = self.position_history[-1][0]
            self.get_logger().debug(
                f'Latest distance: {latest_distance:.2f}m, '
                f'History size: {len(self.position_history)}, '
                f'Status: {status}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PositionHistoryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 