import os
import time
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data


class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__("human_detection_node")
        self.bridge = CvBridge()

        self.subscription_rgb = self.create_subscription(
            Image, "/kinect/image_raw", self.image_callback, qos_profile=qos_profile_sensor_data)
        self.subscription_depth = self.create_subscription(
            Image, "/kinect/depth/image_raw", self.depth_callback, qos_profile=qos_profile_sensor_data)

        self.publisher_position = self.create_publisher(Point, 'rgbd_human_position_sensor', 10)

        self.latest_depth_image = None

        self.onnx_model_path = "/home/isaac/ros2_ws/src/human_detection/human_detection/yolov8n_fp16.onnx"
        if not os.path.exists(self.onnx_model_path):
            self.get_logger().error("ONNX model not found.")
            exit(1)

        self.model = YOLO(self.onnx_model_path)
        self.get_logger().info("YOLO model loaded and node ready.")

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")

    def polar_to_cartesian_from_pixel_depth(self, u, depth, img_width=640, h_fov_deg=57.0):
        """
        Converts horizontal pixel index and depth into Cartesian floor coordinates (x: forward, y: lateral).
        """
        h_fov_rad = np.radians(h_fov_deg)
        angle_offset = (u - img_width / 2) * (h_fov_rad / img_width)

        x = depth * np.cos(angle_offset)  # forward
        y = -depth * np.sin(angle_offset)  # sideways
        return x, y

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        original_height, original_width = frame.shape[:2]
        frame_resized = cv2.resize(frame, (320, 320), interpolation=cv2.INTER_LINEAR)

        results = self.model(frame_resized, imgsz=(320, 320))

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()
                cls = int(box.cls[0].item())

                if cls == 0 and confidence > 0.5:
                    # Rescale box to original image size
                    x1 = int(x1 * (original_width / 320))
                    y1 = int(y1 * (original_height / 320))
                    x2 = int(x2 * (original_width / 320))
                    y2 = int(y2 * (original_height / 320))

                    # Get center of bounding box
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2

                    if self.latest_depth_image is not None:
                        try:
                            depth_mm = self.latest_depth_image[y_center, x_center]
                            if depth_mm > 0:
                                depth_m = depth_mm / 1000.0  # Convert mm to meters

                                x_pos, y_pos = self.polar_to_cartesian_from_pixel_depth(
                                    x_center, depth_m, img_width=original_width, h_fov_deg=57.0
                                )

                                self.get_logger().info(f"Human @ x={x_pos:.2f}m forward, y={y_pos:.2f}m lateral")
                                point_msg = Point(x=x_pos, y=y_pos, z=0.0)
                                self.publisher_position.publish(point_msg)
                            else:
                                self.get_logger().warn("Invalid (zero) depth at center pixel.")
                        except Exception as e:
                            self.get_logger().warn(f"Depth processing failed: {e}")
                    else:
                        self.get_logger().warn("No depth image available.")


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
