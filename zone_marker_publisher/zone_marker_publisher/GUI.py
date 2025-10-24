#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class ZoneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('zone_marker_publisher')

        self.pub = self.create_publisher(Marker, 'zone_marker', 10)
        self.workbench_pub = self.create_publisher(Marker, 'worktop_marker', 10)
        self.sub = self.create_subscription(String, 'closest_link', self.link_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_frame = "base_link"
        self.timer = self.create_timer(0.1, self.publish_markers)  # Increased update rate

        # Publish workbench once at startup
        self.publish_workbench()
        
        self.get_logger().info("🚀 Dynamic zone marker publisher running...")

    def publish_workbench(self):
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

        self.workbench_pub.publish(marker)
        self.get_logger().info("✅ Workbench marker published")

    def link_callback(self, msg):
        if msg.data:  # Only update if we receive a valid frame
            self.get_logger().info(f"🔄 Target frame set to: {msg.data}")
            self.current_frame = msg.data

    def make_circle(self, radius, color, marker_id):
        marker = Marker()
        marker.header.frame_id = self.current_frame  # Set frame to closest link
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "zones"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.01

        # Position at the origin of the current frame
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        return marker

    def publish_markers(self):
        try:
            # Verify that we can transform between base_link and current frame
            self.tf_buffer.lookup_transform('base_link', self.current_frame, rclpy.time.Time())
            
            configs = [
                (0.5, (1.0, 0.0, 0.0, 1.0), 0),  # red
                (2.0, (1.0, 1.0, 0.0, 0.5), 1),  # yellow
                (3.0, (0.0, 1.0, 0.0, 0.2), 2),  # green
            ]

            for radius, color, marker_id in configs:
                marker = self.make_circle(radius, color, marker_id)
                self.pub.publish(marker)

            # Republish workbench periodically to ensure visibility
            self.publish_workbench()

            self.get_logger().debug(f"✅ Published markers centered on frame: {self.current_frame}")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"❌ TF lookup failed for frame '{self.current_frame}': {str(e)}")

def main():
    rclpy.init()
    node = ZoneMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
