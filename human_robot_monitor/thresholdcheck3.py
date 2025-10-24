import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, Float32
import time
import numpy as np
from collections import deque

class HumanPositionComparator(Node):
    def __init__(self):
        super().__init__('human_position_comparator')

        # LiDAR → Kinect transform offsets (tune these!)
        self.lidar_x_offset = 0.4515  # meters
        self.lidar_y_offset = -0.3975  # meters

        # Distance threshold for mismatch
        self.fail_threshold = 0.6  # meters

        # RGBD depth range (min and max distance)
        self.rgbd_view_x_min = 0.58  # near limit
        self.rgbd_view_x_max = 2.6  # far limit

        # RGBD horizontal field of view (Kinect v1 is ~58–60 deg)
        self.rgbd_hfov_deg = 57.0

        # Store recent readings with timestamps
        self.rgbd_readings = deque(maxlen=10)  # (Point, capture_time, publish_time, inference_time)
        self.lidar_readings = deque(maxlen=10)  # (Point, capture_time)

        # Subscriptions
        self.rgbd_sub = self.create_subscription(
            Point, '/rgbd_human_position_sensor', self.rgbd_callback, 10)
        self.get_logger().info('Subscribed to RGBD topic: /rgbd_human_position_sensor')

        self.lidar_sub = self.create_subscription(
            Point, '/lidar_human_position_sensor', self.lidar_callback, 10)
        self.get_logger().info('Subscribed to LIDAR topic: /lidar_human_position_sensor')

        self.inference_time_sub = self.create_subscription(
            Float32, '/inference_time', self.inference_time_callback, 10)
        self.get_logger().info('Subscribed to inference time topic: /inference_time')

        # Detection consistency publisher
        self.consistency_pub = self.create_publisher(Int32, '/detection_consistency', 10)
        self.get_logger().info('Publishing to consistency topic: /detection_consistency')

        # Timer to trigger comparisons
        self.timer = self.create_timer(0.1, self.compare_positions)

        # Latest inference time
        self.latest_inference_time = 0.0  # in seconds
        self.last_rgbd_time = 0.0
        self.last_lidar_time = 0.0

        self.get_logger().info('Node initialized and ready')

    def inference_time_callback(self, msg):
        self.latest_inference_time = msg.data / 1000.0  # Convert ms to seconds
        self.get_logger().info(f'Received inference time: {self.latest_inference_time:.3f}s')

    def rgbd_callback(self, msg):
        current_time = time.time()
        # Calculate capture time using actual inference time
        capture_time = current_time - self.latest_inference_time
        self.rgbd_readings.append((msg, capture_time, current_time, self.latest_inference_time))
        self.last_rgbd_time = current_time
        self.get_logger().info(
            f'RGBD reading received - Position: ({msg.x:.2f}, {msg.y:.2f}), '
            f'Capture time: {capture_time:.3f}, '
            f'Inference time: {self.latest_inference_time:.3f}s'
        )

    def lidar_callback(self, msg):
        current_time = time.time()
        transformed = Point()
        transformed.x = msg.x + self.lidar_x_offset
        transformed.y = msg.y + self.lidar_y_offset
        transformed.z = 0.0
        self.lidar_readings.append((transformed, current_time))
        self.last_lidar_time = current_time
        self.get_logger().info(
            f'LIDAR reading received - Position: ({transformed.x:.2f}, {transformed.y:.2f})'
        )

    def find_matching_readings(self):
        if not self.rgbd_readings:
            self.get_logger().warn('No RGBD readings available')
            return None, None
        if not self.lidar_readings:
            self.get_logger().warn('No LIDAR readings available')
            return None, None

        # Get the most recent RGBD reading
        rgbd_point, rgbd_capture_time, rgbd_publish_time, inference_time = self.rgbd_readings[-1]
        
        # Find the LIDAR reading closest to the RGBD capture time
        best_lidar_reading = None
        min_time_diff = float('inf')
        
        for lidar_point, lidar_time in self.lidar_readings:
            time_diff = abs(lidar_time - rgbd_capture_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                best_lidar_reading = (lidar_point, lidar_time)

        # Only return matches if they're close enough in time
        if min_time_diff > 0.2:  # 200ms maximum time difference
            self.get_logger().warn(
                f'No matching LIDAR reading found. Min time diff: {min_time_diff:.3f}s, '
                f'RGBD capture time: {rgbd_capture_time:.3f}, '
                f'Inference time: {inference_time:.3f}s'
            )
            return None, None

        self.get_logger().info(
            f'Found matching readings - Time diff: {min_time_diff:.3f}s'
        )
        return rgbd_point, best_lidar_reading[0]

    def is_within_rgbd_fov(self, point: Point):
        # 1. Check forward depth range
        if not (self.rgbd_view_x_min <= point.x <= self.rgbd_view_x_max):
            self.get_logger().warn(
                f'Point outside depth range: x={point.x:.2f}m '
                f'(min={self.rgbd_view_x_min:.2f}m, max={self.rgbd_view_x_max:.2f}m)'
            )
            return False

        # 2. Check trapezoidal FOV angle (based on horizontal FOV)
        half_fov_rad = np.radians(self.rgbd_hfov_deg / 2.0)
        max_y_at_x = np.tan(half_fov_rad) * point.x

        if abs(point.y) > max_y_at_x:
            self.get_logger().warn(
                f'Point outside FOV: y={point.y:.2f}m, max_y={max_y_at_x:.2f}m'
            )
            return False

        return True

    def compare_positions(self):
        # Check if we're receiving data
        current_time = time.time()
        if current_time - self.last_rgbd_time > 5.0:
            self.get_logger().warn('No RGBD data received in the last 5 seconds')
        if current_time - self.last_lidar_time > 5.0:
            self.get_logger().warn('No LIDAR data received in the last 5 seconds')

        rgbd_point, lidar_point = self.find_matching_readings()
        
        if rgbd_point is None or lidar_point is None:
            return

        # Skip if LiDAR detection is outside RGBD FOV
        if not self.is_within_rgbd_fov(lidar_point):
            return

        # Compare positions
        dx = rgbd_point.x - lidar_point.x
        dy = rgbd_point.y - lidar_point.y
        distance_diff = np.hypot(dx, dy)

        if distance_diff > self.fail_threshold:
            self.get_logger().error(
                f'SENSOR MISMATCH! Δx={dx:.2f}, Δy={dy:.2f}, Δ={distance_diff:.2f}m '
                f'exceeds threshold {self.fail_threshold:.2f}m'
            )
            self.consistency_pub.publish(Int32(data=0))
        else:
            self.get_logger().info(
                f'Sensors consistent. Δx={dx:.2f}, Δy={dy:.2f}, Δ={distance_diff:.2f}m '
                f'within threshold.'
            )
            self.consistency_pub.publish(Int32(data=1))

def main(args=None):
    rclpy.init(args=args)
    node = HumanPositionComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()