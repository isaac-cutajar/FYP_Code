
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
import numpy as np

class HumanPresenceComparator(Node):
    def __init__(self):
        super().__init__('human_presence_comparator')

        # LiDAR → Kinect transform offsets
        self.lidar_x_offset = 0.4515
        self.lidar_y_offset = -0.3975

        # RGB-D depth range
        self.rgbd_view_x_min = 0.58
        self.rgbd_view_x_max = 2.6

        # RGB-D horizontal FOV (degrees)
        self.rgbd_hfov_deg = 57.0

        # Detection storage
        self.rgbd_point = None
        self.lidar_point = None
        self.last_rgbd_detect_time = 0.0
        self.last_lidar_detect_time = 0.0

        # Grace period in seconds
        self.detection_grace_period = 0.5  # 500 ms

        # Subscriptions
        self.rgbd_sub = self.create_subscription(
            Point, '/rgbd_human_position_sensor', self.rgbd_callback, 10)
        self.lidar_sub = self.create_subscription(
            Point, '/lidar_human_position_sensor', self.lidar_callback, 10)

        # Publisher
        self.consistency_pub = self.create_publisher(Int32, '/detection_consistency', 10)

        # Timer to check consistency
        self.timer = self.create_timer(0.1, self.check_consistency)

        self.get_logger().info('Human Presence Comparator Node initialized with grace period of '
                               f'{self.detection_grace_period}s.')

    def rgbd_callback(self, msg: Point):
        self.rgbd_point = msg
        self.last_rgbd_detect_time = self.get_clock().now().to_sec()

    def lidar_callback(self, msg: Point):
        transformed = Point()
        transformed.x = msg.x + self.lidar_x_offset
        transformed.y = msg.y + self.lidar_y_offset
        transformed.z = 0.0
        self.lidar_point = transformed
        self.last_lidar_detect_time = self.get_clock().now().to_sec()

    def is_within_rgbd_fov(self, point: Point) -> bool:
        # Depth range
        if not (self.rgbd_view_x_min <= point.x <= self.rgbd_view_x_max):
            return False
        # Horizontal FOV
        half_fov_rad = np.radians(self.rgbd_hfov_deg / 2.0)
        max_y = np.tan(half_fov_rad) * point.x
        return abs(point.y) <= max_y

    def check_consistency(self):
        current_time = self.get_clock().now().to_sec()

        rgbd_detected = (self.rgbd_point is not None and
                         current_time - self.last_rgbd_detect_time <= self.detection_grace_period)

        lidar_detected = (self.lidar_point is not None and
                          current_time - self.last_lidar_detect_time <= self.detection_grace_period and
                          self.is_within_rgbd_fov(self.lidar_point))

        if rgbd_detected != lidar_detected:
            # One sensor detects, the other does not → inconsistent
            self.consistency_pub.publish(Int32(data=0))
            self.get_logger().info('Inconsistent: one sensor detects, the other does not.')
        else:
            # Both detect or both do not detect → consistent
            self.consistency_pub.publish(Int32(data=1))
            self.get_logger().info('Consistent: both detect or both do not detect.')

def main(args=None):
    rclpy.init(args=args)
    node = HumanPresenceComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
