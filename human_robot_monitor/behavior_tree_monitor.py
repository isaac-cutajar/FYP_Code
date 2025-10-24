#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float64
from std_srvs.srv import Trigger
from ur_msgs.srv import SetSpeedSliderFraction

class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')
        
        # Initialize status variables
        self.topic_monitor_status = 1.0
        self.threshold_monitor_status = 1.0
        self.safety_monitor_status = 1.0
        self.speed_scaler_status = 1.0
        self.position_history_status = 1.0
        self.speed_scale_value = 1.0
        
        # Create subscribers for all monitor nodes
        self.topic_monitor_sub = self.create_subscription(
            Int32,
            'sensor_status',
            self.topic_monitor_callback,
            10)
            
        self.threshold_monitor_sub = self.create_subscription(
            Int32,
            'detection_consistency',
            self.threshold_monitor_callback,
            10)
            
        self.safety_monitor_sub = self.create_subscription(
            Int32,
            'safety_status',
            self.safety_monitor_callback,
            10)
            
        self.speed_scaler_sub = self.create_subscription(
            Int32,
            'human_detected',
            self.speed_scaler_callback,
            10)
            
        self.position_history_sub = self.create_subscription(
            Int32,
            'human_history_status',
            self.position_history_callback,
            10)
            
        # Subscribe to speed scale value
        self.speed_scale_sub = self.create_subscription(
            Float64,
            '/speed_scale_fraction',
            self.speed_scale_callback,
            10)
            
        # Create service client for robot speed
        self.speed_slider_client = self.create_client(
            SetSpeedSliderFraction, 
            '/io_and_status_controller/set_speed_slider')
            
        # Create service client for robot stop
        self.stop_client = self.create_client(Trigger, '/dashboard_client/pause')
        
        # Timer for periodic behavior tree execution
        self.timer = self.create_timer(0.05, self.execute_behavior_tree)  # 20Hz
        
        self.get_logger().info('Behavior tree node has started')

    def topic_monitor_callback(self, msg):
        self.topic_monitor_status = msg.data
        self.get_logger().debug(f'Topic monitor status updated: {self.topic_monitor_status}')

    def threshold_monitor_callback(self, msg):
        self.threshold_monitor_status = msg.data
        self.get_logger().debug(f'Threshold monitor status updated: {self.threshold_monitor_status}')

    def safety_monitor_callback(self, msg):
        self.safety_monitor_status = msg.data
        self.get_logger().debug(f'Safety monitor status updated: {self.safety_monitor_status}')

    def speed_scaler_callback(self, msg):
        self.speed_scaler_status = msg.data
        self.get_logger().info(f'Speed scaler status updated: {self.speed_scaler_status}')

    def position_history_callback(self, msg):
        self.position_history_status = msg.data
        self.get_logger().debug(f'Position history status updated: {self.position_history_status}')

    def speed_scale_callback(self, msg):
        self.speed_scale_value = msg.data
        self.get_logger().info(f'Speed scale value updated: {self.speed_scale_value}')

    def trigger_robot_stop(self):
        """Send stop command to robot"""
        if not self.stop_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Stop service not available')
            return False
            
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        return True

    def set_robot_speed(self, speed):
        """Set robot speed"""
        if not self.speed_slider_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn('Speed slider service not available')
            return False
            
        request = SetSpeedSliderFraction.Request()
        request.fraction = float(speed)
        future = self.speed_slider_client.call_async(request)
        self.get_logger().info(f'Setting robot speed to: {speed}')
        return True

    def check_inner_selector(self):
        """Check speed_scaler and position_history_monitor in order"""
        self.get_logger().debug(f'Speed scaler status: {self.speed_scaler_status}, Position history status: {self.position_history_status}')
        self.get_logger().debug(f'Current speed scale value: {self.speed_scale_value}')
        
        if self.speed_scaler_status != 0:
            # Speed scaler detected human, use its speed value
            self.get_logger().debug(f'Speed scaler detected human, setting speed to {self.speed_scale_value}')
            self.set_robot_speed(self.speed_scale_value)
            return True
        elif self.position_history_status != 0:
            # Position history detected human, stop robot
            self.get_logger().debug('Position history detected human, stopping robot')
            self.trigger_robot_stop()
            return True
        else:
            # No human detected by either, set full speed
            self.get_logger().debug('No human detected, setting full speed')
            self.set_robot_speed(1.0)
            return False

    def check_sequence(self):
        """Check all conditions in sequence"""
        self.get_logger().debug('Checking sequence...')
        self.get_logger().debug(f'Topic monitor status: {self.topic_monitor_status}')
        self.get_logger().debug(f'Threshold monitor status: {self.threshold_monitor_status}')
        self.get_logger().debug(f'Safety monitor status: {self.safety_monitor_status}')
        
        # Check all monitors - now checking if status is 0 (failure)
        if self.topic_monitor_status == 0:
            self.get_logger().warn(f'Topic monitor check failed: status is 0')
            return False
        if self.threshold_monitor_status == 0:
            self.get_logger().warn(f'Threshold monitor check failed: status is 0')
            return False
        if self.safety_monitor_status == 0:
            self.get_logger().warn(f'Safety monitor check failed: status is 0')
            return False
            
        # Check inner selector
        inner_selector_result = self.check_inner_selector()
        if not inner_selector_result:
            self.get_logger().warn('Inner selector failed: Neither speed_scaler nor position_history detected human')
        return inner_selector_result

    def execute_behavior_tree(self):
        """Execute the behavior tree logic"""
        # Main selector: Try sequence first, then emergency stop
        if not self.check_sequence():
            # Sequence failed, trigger emergency stop
            self.trigger_robot_stop()
            self.get_logger().warn('Sequence failed, triggering emergency stop')
        else:
            self.get_logger().debug('Sequence succeeded, robot operating normally')

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
