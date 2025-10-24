import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time


class HumanPositionComparator(Node):
    def __init__(self):
        super().__init__('human_position_comparator')

        # Offset to align LiDAR origin to Kinect origin
        self.lidar_x_offset = 0.4515  # meters forward
        self.lidar_y_offset = -0.3975  # meters to the left

        self.rgbd_sub = self.create_subscription(
            Point, '/rgbd_human_position', self.rgbd_callback, 10)
        self.lidar_sub = self.create_subscription(
            Point, '/lidar_human_position_sensor', self.lidar_callback, 10)

        self.rgbd_data = None
        self.lidar_data = None
        self.rgbd_time = None
        self.lidar_time = None

        self.timer = self.create_timer(0.5, self.compare_positions)

    def rgbd_callback(self, msg):
        self.rgbd_data = msg
        self.rgbd_time = time.time()

    def lidar_callback(self, msg):
        # Transform lidar coordinates to Kinect frame
        transformed = Point()
        transformed.x = msg.x + self.lidar_x_offset
        transformed.y = msg.y + self.lidar_y_offset
        transformed.z = 0.0
        self.lidar_data = transformed
        self.lidar_time = time.time()

    def compare_positions(self):
        now = time.time()

        if self.rgbd_data is None or (now - self.rgbd_time) > 1.0:
            self.get_logger().warn("No recent RGBD data")
            return

        if self.lidar_data is None or (now - self.lidar_time) > 1.0:
            self.get_logger().warn("No recent LiDAR data")
            return

        dx = self.rgbd_data.x - self.lidar_data.x
        dy = self.rgbd_data.y - self.lidar_data.y
        distance_diff = (dx**2 + dy**2)**0.5

        self.get_logger().info(
            f"Δx = {dx:.2f} m, Δy = {dy:.2f} m, Euclidean diff = {distance_diff:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanPositionComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
