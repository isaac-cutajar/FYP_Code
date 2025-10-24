#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String
import time
from collections import defaultdict

class SpeedScalerNode(Node):
    def __init__(self):
        super().__init__('speed_scaler_node')
        
        # Dictionary to store distances from different sources with timestamps
        self.distances = defaultdict(lambda: {'value': None, 'timestamp': None})
        
        # Data timeout in seconds
        self.data_timeout = 0.5  # 500ms timeout
        
        # Define distance thresholds and speed scales
        self.min_distance = 0.5  # meters
        self.max_distance = 2.0  # meters
        self.min_speed = 0.1     # 10% speed
        self.max_speed = 1.0     # 100% speed
        
        # Create subscribers for distances from different sources
        self.lidar_distance_sub = self.create_subscription(
            Float64, 
            '/lidar_human_distance',
            lambda msg: self.distance_callback(msg, 'lidar'),
            10)
            
        self.rgbd_distance_sub = self.create_subscription(
            Float64, 
            '/rgbd_human_distance',
            lambda msg: self.distance_callback(msg, 'rgbd'),
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

        # Add publisher for shortest distance with source
        self.shortest_distance_pub = self.create_publisher(
            String,
            'shortest_distance',
            10)
            
        # Timer for periodic speed update
        self.timer = self.create_timer(0.05, self.update_speed)  # 20Hz update
        
        self.get_logger().info('Speed scaler node has started')

    def is_data_valid(self, data):
        """Check if data entry is still valid based on timestamp"""
        if data['value'] is None or data['timestamp'] is None:
            return False
        return (time.time() - data['timestamp']) < self.data_timeout

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

    def distance_callback(self, msg, source):
        """Process distance data"""
        self.distances[source] = {
            'value': msg.data,
            'timestamp': time.time()
        }

    def get_min_valid_distance(self):
        """Get the minimum distance from all valid sources with source information"""
        valid_distances = []
        current_time = time.time()
        
        # Collect all valid distances with their sources
        for source, data in self.distances.items():
            if self.is_data_valid(data):
                valid_distances.append((data['value'], source))
            
        if valid_distances:
            # Find minimum distance and its source
            min_distance, source = min(valid_distances, key=lambda x: x[0])
            return min_distance, source
        return None, None

    def update_speed(self):
        """Update robot speed based on minimum distance"""
        # Get minimum valid distance and its source
        min_distance, source = self.get_min_valid_distance()
        
        # Prepare message for shortest distance
        distance_msg = String()
        
        # Check if we have any valid distances
        if min_distance is None:
            speed_scale = self.max_speed
            detection_status = 0
            distance_msg.data = "No Human Detected"
        else:
            # Calculate speed scale based on minimum distance
            speed_scale = self.calculate_speed_scale(min_distance)
            detection_status = 1
            distance_msg.data = f"{source.upper()}: {min_distance:.2f}m"
        
        # Publish speed scale
        speed_msg = Float64()
        speed_msg.data = float(speed_scale)
        self.speed_pub.publish(speed_msg)
        
        # Publish detection status
        status_msg = Int32()
        status_msg.data = detection_status
        self.detection_pub.publish(status_msg)
        
        # Publish shortest distance with source
        self.shortest_distance_pub.publish(distance_msg)
        
        # Log for debugging
        self.get_logger().debug(
            f'Shortest distance: {distance_msg.data}, '
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
