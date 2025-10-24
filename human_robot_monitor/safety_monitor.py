#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Int32
import numpy as np
import math
import time

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Initialize position and speed variables
        self.lidar_position = None
        self.rgbd_position = None
        self.lidar_speed = None
        self.rgbd_speed = None
        
        # Add timestamps for data timeout
        self.lidar_position_time = None
        self.rgbd_position_time = None
        self.lidar_speed_time = None
        self.rgbd_speed_time = None
        
        # Data timeout in seconds
        self.data_timeout = 0.5  # 500ms timeout
        
        # Safety parameters (to be set via parameters)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('reaction_time', 0.1),          # System reaction time in seconds
                ('robot_stopping_time', 0.1),    # Robot stopping time in seconds
                ('intrusion_distance', 0.1),     # Intrusion distance C in meters
                ('position_uncertainty', 0.05),   # Position uncertainty Zd in meters
                ('robot_uncertainty', 0.05),      # Robot position uncertainty Zr in meters
                ('robot_reaction_distance', 0.1), # Distance moved by robot during reaction Sr in meters
                ('robot_stopping_distance', 0.1), # Distance moved by robot while stopping Ss in meters
            ]
        )
        
        # Create subscribers
        self.lidar_pos_sub = self.create_subscription(
            Point, 
            '/lidar_human_position',
            self.lidar_position_callback,
            10)
            
        self.rgbd_pos_sub = self.create_subscription(
            Point,
            '/rgbd_human_position',
            self.rgbd_position_callback,
            10)
            
        self.lidar_speed_sub = self.create_subscription(
            Vector3,
            '/lidar_human_speed',
            self.lidar_speed_callback,
            10)
            
        self.rgbd_speed_sub = self.create_subscription(
            Vector3,
            '/rgbd_human_speed',
            self.rgbd_speed_callback,
            10)
            
        # Create publisher for safety status
        self.safety_pub = self.create_publisher(Int32, 'safety_status', 10)
        
        # Timer for periodic safety check
        self.timer = self.create_timer(0.05, self.check_safety)  # 20Hz check
        
        self.get_logger().info('Safety monitor node has started')

    def lidar_position_callback(self, msg):
        self.lidar_position = msg
        self.lidar_position_time = time.time()
        self.get_logger().debug(f"Received LIDAR position: x={msg.x}, y={msg.y}")

    def rgbd_position_callback(self, msg):
        self.rgbd_position = msg
        self.rgbd_position_time = time.time()
        self.get_logger().debug(f"Received RGBD position: x={msg.x}, y={msg.y}")

    def lidar_speed_callback(self, msg):
        self.lidar_speed = msg
        self.lidar_speed_time = time.time()
        self.get_logger().debug(f"Received LIDAR speed: x={msg.x}, y={msg.y}, z={msg.z}")

    def rgbd_speed_callback(self, msg):
        self.rgbd_speed = msg
        self.rgbd_speed_time = time.time()
        self.get_logger().debug(f"Received RGBD speed: x={msg.x}, y={msg.y}, z={msg.z}")

    def is_data_valid(self, data, timestamp):
        if data is None or timestamp is None:
            return False
        return (time.time() - timestamp) < self.data_timeout

    def calculate_protective_distance(self, human_speed):
        # Get parameters
        Tr = self.get_parameter('reaction_time').value
        Ts = self.get_parameter('robot_stopping_time').value
        C = self.get_parameter('intrusion_distance').value
        Zd = self.get_parameter('position_uncertainty').value
        Zr = self.get_parameter('robot_uncertainty').value
        Sr = self.get_parameter('robot_reaction_distance').value
        Ss = self.get_parameter('robot_stopping_distance').value
        
        # Calculate components
        Sh = human_speed * (Tr + Ts)  # Distance covered by human
        
        # Calculate total protective separation distance using constant Sr and Ss
        Sp = Sh + Sr + Ss + C + Zd + Zr
        
        return Sp

    def get_closest_distance(self):
        current_time = time.time()
        
        # Check if data is too old
        if (self.lidar_position is None or 
            self.lidar_position_time is None or 
            (current_time - self.lidar_position_time) > self.data_timeout):
            self.lidar_position = None
            
        if (self.rgbd_position is None or 
            self.rgbd_position_time is None or 
            (current_time - self.rgbd_position_time) > self.data_timeout):
            self.rgbd_position = None
            
        if self.lidar_position is None and self.rgbd_position is None:
            return None
            
        distances = []
        
        if self.lidar_position is not None:
            lidar_dist = math.sqrt(
                self.lidar_position.x ** 2 + 
                self.lidar_position.y ** 2
            )
            distances.append(lidar_dist)
            
        if self.rgbd_position is not None:
            rgbd_dist = math.sqrt(
                self.rgbd_position.x ** 2 + 
                self.rgbd_position.y ** 2
            )
            distances.append(rgbd_dist)
            
        return min(distances) if distances else None

    def get_highest_speed(self):
        current_time = time.time()
        
        # Check if data is too old
        if (self.lidar_speed is None or 
            self.lidar_speed_time is None or 
            (current_time - self.lidar_speed_time) > self.data_timeout):
            self.lidar_speed = None
            
        if (self.rgbd_speed is None or 
            self.rgbd_speed_time is None or 
            (current_time - self.rgbd_speed_time) > self.data_timeout):
            self.rgbd_speed = None
            
        if self.lidar_speed is None and self.rgbd_speed is None:
            return None
            
        speeds = []
        
        if self.lidar_speed is not None:
            speeds.append(abs(self.lidar_speed.x))
            
        if self.rgbd_speed is not None:
            speeds.append(abs(self.rgbd_speed.x))
            
        return max(speeds) if speeds else None

    def check_safety(self):
        current_distance = self.get_closest_distance()
        current_speed = self.get_highest_speed()
        
        if current_distance is None or current_speed is None:
            # Only log debug message if we haven't received data for a while
            if (self.lidar_position_time is None or time.time() - self.lidar_position_time > self.data_timeout) and \
               (self.rgbd_position_time is None or time.time() - self.rgbd_position_time > self.data_timeout):
                self.get_logger().debug(
                    f'No valid position data received for {self.data_timeout}s\n'
                    f'LIDAR position: {self.lidar_position}\n'
                    f'RGBD position: {self.rgbd_position}'
                )
            return
            
        # Calculate protective separation distance
        protective_distance = self.calculate_protective_distance(current_speed)
        
        # Check if current distance is safe
        is_safe = 1 if current_distance > protective_distance else 0
        
        # Publish safety status
        status_msg = Int32()
        status_msg.data = is_safe
        self.safety_pub.publish(status_msg)
        
        # Log for debugging
        self.get_logger().debug(
            f'Distance: {current_distance:.3f}m, '
            f'Human Speed: {current_speed:.3f}m/s, '
            f'Required separation: {protective_distance:.3f}m, '
            f'Safe: {is_safe}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 