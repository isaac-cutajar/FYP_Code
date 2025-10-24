#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
import time

class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('topic_monitor_node')
        
        # Initialize timestamps for each topic
        self.last_scan_time = 0.0
        self.last_rgb_time = 0.0
        self.last_depth_time = 0.0
        
        # Track when topics first became inactive
        self.scan_inactive_since = 0.0
        self.rgb_inactive_since = 0.0
        self.depth_inactive_since = 0.0
        
        # Create subscribers with sensor data QoS profile
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)
            
        self.rgb_sub = self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data)
            
        self.depth_sub = self.create_subscription(
            Image,
            '/kinect/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data)
            
        # Create publisher for status
        self.status_pub = self.create_publisher(Int32, 'sensor_status', 10)
        
        # Create timer for checking topic status
        self.timer = self.create_timer(0.1, self.check_topics)  # Check every 100ms
        
        # Timeout threshold for considering a topic as inactive (increased from 1.0 to 2.0 seconds)
        self.timeout_threshold = 2.0
        
        # Grace period for how long a topic can be inactive before affecting overall status (5 seconds)
        self.grace_period = 5.0
        
        self.get_logger().info('Topic monitor node has started')

    def scan_callback(self, msg):
        self.last_scan_time = time.time()

    def rgb_callback(self, msg):
        self.last_rgb_time = time.time()

    def depth_callback(self, msg):
        self.last_depth_time = time.time()

    def check_topics(self):
        current_time = time.time()
        
        # Check if topics are active (received within timeout threshold)
        scan_active = (current_time - self.last_scan_time) < self.timeout_threshold
        rgb_active = (current_time - self.last_rgb_time) < self.timeout_threshold
        depth_active = (current_time - self.last_depth_time) < self.timeout_threshold
        
        # Update inactive timestamps
        if not scan_active and self.scan_inactive_since == 0.0:
            self.scan_inactive_since = current_time
        elif scan_active:
            self.scan_inactive_since = 0.0
            
        if not rgb_active and self.rgb_inactive_since == 0.0:
            self.rgb_inactive_since = current_time
        elif rgb_active:
            self.rgb_inactive_since = 0.0
            
        if not depth_active and self.depth_inactive_since == 0.0:
            self.depth_inactive_since = current_time
        elif depth_active:
            self.depth_inactive_since = 0.0
        
        # Check if any topic has been inactive for longer than the grace period
        scan_failed = not scan_active and (current_time - self.scan_inactive_since) > self.grace_period
        rgb_failed = not rgb_active and (current_time - self.rgb_inactive_since) > self.grace_period
        depth_failed = not depth_active and (current_time - self.depth_inactive_since) > self.grace_period
        
        # Create status message - consider system active if no topic has failed
        status_msg = Int32()
        status_msg.data = 0 if (scan_failed or rgb_failed or depth_failed) else 1
        
        # Publish status
        self.status_pub.publish(status_msg)
        
        # Log status for debugging
        if not (scan_active and rgb_active and depth_active):
            self.get_logger().info(
                f'Topic status - Scan: {scan_active} ({self.scan_inactive_since:.1f}s), '
                f'RGB: {rgb_active} ({self.rgb_inactive_since:.1f}s), '
                f'Depth: {depth_active} ({self.depth_inactive_since:.1f}s)'
            )

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 