#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import threading
import time


class ZoneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('zone_marker_publisher')

        self.pub = self.create_publisher(Marker, 'zone_marker', 10)
        self.worktop_pub = self.create_publisher(Marker, 'worktop_marker', 10)
        self.sub = self.create_subscription(String, 'closest_link', self.link_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_frame = "base_link"

        # Publish worktop marker once after startup (in separate thread so it doesn't block)
        threading.Thread(target=self.publish_worktop_once, daemon=True).start()

        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info("🚀 Dynamic zone marker publisher running...")

    def link_callback(self, msg):
        self.get_logger().info(f"🔄 Target frame set to: {msg.data}")
        self.current_frame = msg.data

    def make_circle(self, radius, color, marker_id, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "zones"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.01

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z  # z-level set dynamically

        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        return marker

    def publish_markers(self):
        try:
            tf = self.tf_buffer.lookup_transform('base_link', self.current_frame, rclpy.time.Time())

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            self.get_logger().info(f"✅ Drawing at {self.current_frame}: x={x:.2f}, y={y:.2f}")

            configs = [
                (3.0, (0.0, 1.0, 0.0, 1.0), 2, -1.02),  # Green, bottom
                (2.0, (1.0, 1.0, 0.0, 1.0), 1, -1.01),  # Yellow, middle
                (0.5, (1.0, 0.0, 0.0, 1.0), 0, -1.00),  # Red, top
            ]

            for radius, color, marker_id, z in configs:
                marker = self.make_circle(radius, color, marker_id, x, y, z)
                self.pub.publish(marker)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"❌ TF lookup failed for frame '{self.current_frame}': {str(e)}")

    def publish_worktop_once(self):
        # Wait a little to allow subscribers (e.g. RViz) to connect
        time.sleep(1.0)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "worktop"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.scale.x = 1.3  # length
        marker.scale.y = 0.6  # width
        marker.scale.z = 1.0  # height

        marker.pose.position.x = -0.545
        marker.pose.position.y = 0.2
        marker.pose.position.z = -0.5
        marker.pose.orientation.w = 1.0

        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        self.worktop_pub.publish(marker)
        self.get_logger().info("✅ Worktop marker sent once.")


def main():
    rclpy.init()
    node = ZoneMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
