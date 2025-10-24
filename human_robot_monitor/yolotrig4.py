import os
import time
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, TransformStamped
from rclpy.qos import qos_profile_sensor_data
import tf2_ros


class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__("human_detection_node")
        self.bridge = CvBridge()

        self.subscription_rgb = self.create_subscription(
            Image, "/kinect/image_raw", self.image_callback, qos_profile=qos_profile_sensor_data)
        self.subscription_depth = self.create_subscription(
            Image, "/kinect/depth/image_raw", self.depth_callback, qos_profile=qos_profile_sensor_data)

        # Publishers for different position representations
        self.publisher_position_sensor = self.create_publisher(Point, 'rgbd_human_position_sensor', 10)
        self.publisher_position_link = self.create_publisher(Point, 'rgbd_human_position', 10)
        self.publisher_distance = self.create_publisher(Float64, 'rgbd_human_distance', 10)

        # Depth image buffer to handle occasional missing depth data
        self.latest_depth_image = None
        self.depth_buffer = []
        self.max_buffer_size = 5  # Keep the last 5 depth images
        self.last_depth_time = 0.0
        self.depth_timeout = 0.5  # 500ms timeout for depth data

        self.onnx_model_path = "~/ros2_ws/src/human_detection/human_detection/yolov8n_fp16.onnx"
        if not os.path.exists(self.onnx_model_path):
            self.get_logger().error("ONNX model not found.")
            exit(1)

        self.model = YOLO(self.onnx_model_path)
        
        # TF buffer and listener for robot link transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Static transform broadcaster: base_link → kinect
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_transform()
        
        # Robot links to monitor
        self.robot_links = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "flange",
            "tool0"
        ]
        
        self.get_logger().info("Yolotrig3: YOLO model loaded and node ready.")
        
    def broadcast_static_transform(self):
        """
        Broadcast a static transform from base_link to kinect.
        This establishes the relationship between the robot's base_link and the Kinect sensor.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'kinect'
        
        # Set the translation (position) of the Kinect relative to the base_link
        # These values should be adjusted based on your actual setup
        t.transform.translation.x = 0.115  # Adjust based on your setup
        t.transform.translation.y = -20.5  # Adjust based on your setup
        t.transform.translation.z = 0.6  # Adjust based on your setup
        
        # Set the rotation (orientation) of the Kinect relative to the base_link
        # Using a simple rotation around the Z axis (90 degrees)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0
        
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info("Static transform from base_link to kinect broadcasted.")

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_image = depth_image
            self.last_depth_time = time.time()
            
            # Add to buffer
            self.depth_buffer.append(depth_image)
            if len(self.depth_buffer) > self.max_buffer_size:
                self.depth_buffer.pop(0)  # Remove oldest image
                
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")

    def polar_to_cartesian_from_pixel_depth(self, u, v, depth, img_width=640, img_height=480, h_fov_deg=57.0, v_fov_deg=43.0):
        """
        Converts pixel coordinates and depth into Cartesian coordinates (x: forward, y: lateral, z: up).
        """
        h_fov_rad = np.radians(h_fov_deg)
        v_fov_rad = np.radians(v_fov_deg)
        
        # Calculate horizontal angle
        h_angle_offset = (u - img_width / 2) * (h_fov_rad / img_width)
        
        # Calculate vertical angle
        v_angle_offset = (v - img_height / 2) * (v_fov_rad / img_height)
        
        # Calculate coordinates
        x = depth * np.cos(h_angle_offset) * np.cos(v_angle_offset)  # forward
        y = -depth * np.sin(h_angle_offset) * np.cos(v_angle_offset)  # sideways
        z = depth * np.sin(v_angle_offset)  # up
        
        return x, y, z
        
    def find_closest_robot_link(self, x_pos, y_pos, z_pos):
        """
        Find the closest robot link to the given position using full 3D distance.
        Returns the closest link name, its 3D position, relative position, and the distance.
        """
        closest_dist = float('inf')
        closest_link = None
        closest_link_pos = None
        closest_rel_pos = None
        
        for link in self.robot_links:
            try:
                # Look up transform from base_link to the robot link
                tf = self.tf_buffer.lookup_transform("base_link", link, rclpy.time.Time())
                lx = tf.transform.translation.x
                ly = tf.transform.translation.y
                lz = tf.transform.translation.z
                
                # Calculate relative position and distance
                rel_pos = (x_pos - lx, y_pos - ly, z_pos - lz)
                dist = np.linalg.norm(rel_pos)
                
                if dist < closest_dist:
                    closest_dist = dist
                    closest_link = link
                    closest_link_pos = (lx, ly, lz)
                    closest_rel_pos = rel_pos
            except Exception as e:
                self.get_logger().warn(f"Error looking up transform for {link}: {str(e)}")
                continue
                
        return closest_link, closest_link_pos, closest_rel_pos, closest_dist

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

                if cls == 0 and confidence > 0.7:  # Person class with confidence > 0.7
                    # Rescale box to original image size
                    x1 = int(x1 * (original_width / 320))
                    y1 = int(y1 * (original_height / 320))
                    x2 = int(x2 * (original_width / 320))
                    y2 = int(y2 * (original_height / 320))

                    if self.latest_depth_image is not None:
                        try:
                            # Extract the region of interest from the depth image
                            roi = self.latest_depth_image[y1:y2, x1:x2]
                            
                            # Find the minimum depth value in the ROI (excluding zeros)
                            valid_depths = roi[roi > 0]
                            if len(valid_depths) > 0:
                                min_depth_mm = np.min(valid_depths)
                                min_depth_m = min_depth_mm / 1000.0  # Convert mm to meters
                                
                                # Find the position of the minimum depth value
                                min_depth_pos = np.unravel_index(np.argmin(roi), roi.shape)
                                min_depth_y, min_depth_x = min_depth_pos
                                
                                # Convert to original image coordinates
                                x_center = x1 + min_depth_x
                                y_center = y1 + min_depth_y
                                
                                # Convert to Cartesian coordinates with z-coordinate
                                x_pos, y_pos, z_pos = self.polar_to_cartesian_from_pixel_depth(
                                    x_center, y_center, min_depth_m, 
                                    img_width=original_width, img_height=original_height, 
                                    h_fov_deg=57.0, v_fov_deg=43.0
                                )
                                
                                # Publish position relative to sensor
                                sensor_point_msg = Point(x=x_pos, y=y_pos, z=z_pos)
                                self.publisher_position_sensor.publish(sensor_point_msg)
                                
                                # Find closest robot link using full 3D position
                                closest_link, closest_link_pos, rel_pos, min_distance = self.find_closest_robot_link(x_pos, y_pos, z_pos)
                                
                                if closest_link is not None and closest_link_pos is not None:
                                    # Use pre-calculated relative position
                                    rel_x, rel_y, rel_z = rel_pos
                                    
                                    # Publish position relative to closest link
                                    link_point_msg = Point(x=rel_x, y=rel_y, z=rel_z)
                                    self.publisher_position_link.publish(link_point_msg)
                                    
                                    # Publish distance to closest link
                                    distance_msg = Float64()
                                    distance_msg.data = float(min_distance)
                                    self.publisher_distance.publish(distance_msg)
                                    
                                    self.get_logger().info(
                                        f"Human @ x={x_pos:.2f}m forward, y={y_pos:.2f}m lateral, z={z_pos:.2f}m up (min depth: {min_depth_m:.2f}m), "
                                        f"Closest link: {closest_link}, Distance={min_distance:.2f}m, "
                                        f"Relative position=({rel_x:.2f}, {rel_y:.2f}, {rel_z:.2f})m"
                                    )
                                else:
                                    self.get_logger().warn("Could not find closest robot link.")
                            else:
                                self.get_logger().warn("No valid depth values in the bounding box.")
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
