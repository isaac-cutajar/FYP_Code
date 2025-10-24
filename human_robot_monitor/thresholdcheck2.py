import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import time
import numpy as np


class HumanPositionComparator(Node):
    def __init__(self):
        super().__init__('human_position_comparator')

        # LiDAR → Kinect transform offsets (tune these!)
        self.lidar_x_offset = 0.4515  # meters
        self.lidar_y_offset = -0.3975  # meters

        # Distance threshold for mismatch
        self.fail_threshold = 5.0  # meters

        # RGBD depth range (min and max distance)
        self.rgbd_view_x_min = 0.58  # near limit
        self.rgbd_view_x_max = 2.6  # far limit

        # RGBD horizontal field of view (Kinect v1 is ~58–60 deg)
        self.rgbd_hfov_deg = 57.0

        # Latest data + timestamps
        self.rgbd_data = None  # (Point, timestamp)
        self.lidar_data = None  # (Point, timestamp)

        # Subscriptions
        self.rgbd_sub = self.create_subscription(
            Point, '/rgbd_human_position_sensor', self.rgbd_callback, 10)

        self.lidar_sub = self.create_subscription(
            Point, '/lidar_human_position_sensor', self.lidar_callback, 10)

        # Detection consistency publisher
        self.consistency_pub = self.create_publisher(Int32, '/detection_consistency', 10)

        # Timer to trigger comparisons
        self.timer = self.create_timer(1.0, self.compare_positions)

    def rgbd_callback(self, msg):
        self.rgbd_data = (msg, time.time())

    def lidar_callback(self, msg):
        transformed = Point()
        transformed.x = msg.x + self.lidar_x_offset
        transformed.y = msg.y + self.lidar_y_offset
        transformed.z = 0.0
        self.lidar_data = (transformed, time.time())

    def is_within_rgbd_fov(self, point: Point):
        # 1. Check forward depth range
        if not (self.rgbd_view_x_min <= point.x <= self.rgbd_view_x_max):
            return False

        # 2. Check trapezoidal FOV angle (based on horizontal FOV)
        half_fov_rad = np.radians(self.rgbd_hfov_deg / 2.0)
        max_y_at_x = np.tan(half_fov_rad) * point.x

        return abs(point.y) <= max_y_at_x

    def compare_positions(self):
        if self.rgbd_data is None:
            self.get_logger().warn("No RGBD data received yet")
            return
        if self.lidar_data is None:
            self.get_logger().warn("No LiDAR data received yet")
            return

        rgbd_point, t_rgbd = self.rgbd_data
        lidar_point, t_lidar = self.lidar_data
        now = time.time()

        # Skip stale data
        if now - t_rgbd > 2.0:
            self.get_logger().warn("RGBD data too old")
            return
        if now - t_lidar > 2.0:
            self.get_logger().warn("LiDAR data too old")
            return

        # Wait until timestamps are roughly synced
        if abs(t_rgbd - t_lidar) > 0.5:
            self.get_logger().info("Waiting for synced RGBD and LiDAR data...")
            return

        # Skip if LiDAR detection is outside RGBD FOV
        if not self.is_within_rgbd_fov(lidar_point):
            self.get_logger().info("LiDAR detection outside RGBD FOV — skipping check.")
            return

        # Compare positions
        dx = rgbd_point.x - lidar_point.x
        dy = rgbd_point.y - lidar_point.y
        distance_diff = np.hypot(dx, dy)

        if distance_diff > self.fail_threshold:
            self.get_logger().error(
                f"SENSOR MISMATCH! Δx={dx:.2f}, Δy={dy:.2f}, Δ={distance_diff:.2f}m exceeds threshold {self.fail_threshold:.2f}m"
            )
            self.consistency_pub.publish(Int32(data=0))
        else:
            self.get_logger().info(
                f"Sensors consistent. Δx={dx:.2f}, Δy={dy:.2f}, Δ={distance_diff:.2f}m within threshold."
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
