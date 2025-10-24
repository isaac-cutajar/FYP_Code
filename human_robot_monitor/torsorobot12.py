import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Point, Vector3
from std_msgs.msg import String, Float64, Int32
from time import time
import tf2_ros

class BackgroundSubtractionNode(Node):
    def __init__(self):
        super().__init__('bg_subtraction_people_tracking')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.publisher_centroids = self.create_publisher(MarkerArray, 'centroid_markers', 10)
        self.publisher_closest_position = self.create_publisher(Point, 'lidar_human_position', 10)
        self.publisher_closest_position_sensor = self.create_publisher(Point, 'lidar_human_position_sensor', 10)
        self.publisher_closest_speed = self.create_publisher(Vector3, 'lidar_human_speed', 10)
        self.publisher_closest_distance = self.create_publisher(Float64, 'lidar_human_distance', 10)
        self.publisher_closest_link = self.create_publisher(String, 'closest_link', 10)
        self.publisher_human_detected = self.create_publisher(Int32, 'human_detected', 10)

        # Define exclusion zone (in laser frame)
        self.exclusion_zone = {
            'min_x': -0.3,  # meters
            'max_x': 0.01,
            'min_y': 0.18,
            'max_y': 0.60
        }

        self.bg_frames = []
        self.bg_collection_frames = 30
        self.static_background = None
        self.bg_initialized = False

        self.max_range = 2.5
        self.dynamic_threshold = 0.1
        self.cluster_distance_threshold = 0.4  # Increased for torso detection
        self.min_cluster_points = 30  # Minimum points for a valid cluster
        self.max_cluster_points = 200  # Maximum points for a valid cluster

        self.frame_count = 0
        self.clear_interval = 10

        # Tracked people: list of dicts with 'x', 'y', 'time', 'vx', 'vy'
        self.tracked_people = []
        self.tracking_timeout = 1.5  # seconds

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Static transform broadcaster: base_link → laser
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

    def broadcast_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser'
        t.transform.translation.x = 0.1925
        t.transform.translation.y = 0.3365
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.70710678
        t.transform.rotation.w = 0.70710678
        self.static_broadcaster.sendTransform(t)

    def point_in_exclusion_zone(self, x, y):
        """Check if a point falls within the exclusion zone."""
        return (self.exclusion_zone['min_x'] <= x <= self.exclusion_zone['max_x'] and
                self.exclusion_zone['min_y'] <= y <= self.exclusion_zone['max_y'])

    def scan_callback(self, msg):
        ranges_np = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges_np))
        valid = (ranges_np > msg.range_min) & (ranges_np < msg.range_max)
        ranges_np[~valid] = np.nan

        if not self.bg_initialized:
            self.bg_frames.append(ranges_np.copy())
            self.get_logger().info(f"Collecting background frame {len(self.bg_frames)}/{self.bg_collection_frames}")
            if len(self.bg_frames) >= self.bg_collection_frames:
                self.static_background = np.nanmedian(np.array(self.bg_frames), axis=0)
                self.bg_initialized = True
                self.get_logger().info("Static background initialized.")
            return

        self.frame_count += 1
        if self.frame_count % self.clear_interval == 0:
            self.clear_markers(self.publisher_centroids)

        dynamic_points = []
        current_time = time()

        for i in range(len(ranges_np)):
            angle = angles[i]
            range_curr = ranges_np[i]
            range_bg = self.static_background[i]

            if np.isnan(range_curr) or np.isnan(range_bg):
                continue
            if range_curr > self.max_range:
                continue

            x = range_curr * np.cos(angle)
            y = range_curr * np.sin(angle)
            
            # Skip points in the exclusion zone
            if self.point_in_exclusion_zone(x, y):
                continue
                
            x_bg = range_bg * np.cos(angle)
            y_bg = range_bg * np.sin(angle)

            xy_dist = np.linalg.norm([x - x_bg, y - y_bg])
            if xy_dist >= self.dynamic_threshold:
                dynamic_points.append((x, y))

        clusters, centroids = self.cluster_dynamic_points(dynamic_points)
        # Filter clusters based on point count range
        valid_indices = [i for i, c in enumerate(clusters) if self.min_cluster_points <= len(c) <= self.max_cluster_points]
        valid_clusters = [clusters[i] for i in valid_indices]
        valid_centroids = [centroids[i] for i in valid_indices]

        # Update tracking with velocity
        new_tracked = []
        for person_pos in valid_centroids:
            matched = False
            for person in self.tracked_people:
                dist = np.linalg.norm(person_pos - np.array([person['x'], person['y']]))
                if dist < 0.5:
                    dt = current_time - person['time']
                    if dt > 0:
                        vx = (person_pos[0] - person['x']) / dt
                        vy = (person_pos[1] - person['y']) / dt
                    else:
                        vx = vy = 0.0
                    new_tracked.append({
                        'x': person_pos[0],
                        'y': person_pos[1],
                        'time': current_time,
                        'vx': vx,
                        'vy': vy
                    })
                    matched = True
                    break
            if not matched:
                new_tracked.append({
                    'x': person_pos[0],
                    'y': person_pos[1],
                    'time': current_time,
                    'vx': 0.0,
                    'vy': 0.0
                })

        self.tracked_people = [p for p in new_tracked if current_time - p['time'] < self.tracking_timeout]

        # Find closest person to robot
        closest_person = None
        min_distance = float('inf')
        closest_link = None
        closest_link_pos = None
        
        for person in self.tracked_people:
            # Find closest link to this person
            person_closest_dist = float('inf')
            person_closest_link = None
            person_closest_link_pos = None
            
            for link in self.robot_links:
                try:
                    tf = self.tf_buffer.lookup_transform("laser", link, rclpy.time.Time())
                    lx = tf.transform.translation.x
                    ly = tf.transform.translation.y
                    dist = np.linalg.norm([person['x'] - lx, person['y'] - ly])
                    if dist < person_closest_dist:
                        person_closest_dist = dist
                        person_closest_link = link
                        person_closest_link_pos = (lx, ly)
                except Exception as e:
                    self.get_logger().warn(f"Error looking up transform for {link}: {str(e)}")
                    continue
            
            # Update global closest if this person is closer
            if person_closest_dist < min_distance:
                min_distance = person_closest_dist
                closest_person = person
                closest_link = person_closest_link
                closest_link_pos = person_closest_link_pos

        # Publish closest person data if found
        if closest_person is not None and closest_link_pos is not None:
            # Calculate relative position to closest link
            rel_x = closest_person['x'] - closest_link_pos[0]
            rel_y = closest_person['y'] - closest_link_pos[1]
            
            # Calculate radial velocity
            r_vec = np.array([rel_x, rel_y])
            if np.linalg.norm(r_vec) > 0:
                radial_velocity = np.dot([closest_person['vx'], closest_person['vy']], r_vec) / np.linalg.norm(r_vec)
            else:
                radial_velocity = 0.0
            
            # Publish human detection status (1 = detected)
            detection_msg = Int32()
            detection_msg.data = 1
            self.publisher_human_detected.publish(detection_msg)
            
            # Publish position relative to closest link
            position_msg = Point()
            position_msg.x = float(rel_x)
            position_msg.y = float(rel_y)
            position_msg.z = 0.0
            self.publisher_closest_position.publish(position_msg)

            # Publish position relative to sensor
            sensor_position_msg = Point()
            sensor_position_msg.x = float(closest_person['x'])
            sensor_position_msg.y = float(closest_person['y'])
            sensor_position_msg.z = 0.0
            self.publisher_closest_position_sensor.publish(sensor_position_msg)

            # Publish radial velocity
            velocity_msg = Vector3()
            velocity_msg.x = float(radial_velocity)
            velocity_msg.y = 0.0
            velocity_msg.z = 0.0
            self.publisher_closest_speed.publish(velocity_msg)

            # Publish distance to closest link
            distance_msg = Float64()
            distance_msg.data = float(min_distance)
            self.publisher_closest_distance.publish(distance_msg)

            # Publish closest link name
            link_name = String()
            link_name.data = closest_link
            self.publisher_closest_link.publish(link_name)

            self.get_logger().info(
                f"Closest person to {closest_link}: Distance={min_distance:.2f}m, "
                f"Relative position=({rel_x:.2f}, {rel_y:.2f})m, "
                f"Radial velocity={radial_velocity:.2f} m/s"
            )
        else:
            self.get_logger().warn("No person detected to publish position/speed")
            # Publish human detection status (0 = not detected)
            detection_msg = Int32()
            detection_msg.data = 0
            self.publisher_human_detected.publish(detection_msg)

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, person in enumerate(self.tracked_people):
            x, y = person['x'], person['y']
            vx, vy = person['vx'], person['vy']

            # Visual markers
            body = Marker()
            body.header.frame_id = "laser"
            body.header.stamp = stamp
            body.id = i * 2
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = float(x)
            body.pose.position.y = float(y)
            body.pose.position.z = 0.75
            body.scale.x = 0.3
            body.scale.y = 0.3
            body.scale.z = 1.5
            body.color.a = 1.0
            body.color.r = 0.0
            body.color.g = 0.5
            body.color.b = 1.0
            marker_array.markers.append(body)

            head = Marker()
            head.header.frame_id = "laser"
            head.header.stamp = stamp
            head.id = i * 2 + 1
            head.type = Marker.SPHERE
            head.action = Marker.ADD
            head.pose.position.x = float(x)
            head.pose.position.y = float(y)
            head.pose.position.z = 1.65
            head.scale.x = 0.3
            head.scale.y = 0.3
            head.scale.z = 0.3
            head.color.a = 1.0
            head.color.r = 0.0
            head.color.g = 0.5
            head.color.b = 1.0
            marker_array.markers.append(head)

            # Closest link
            closest_dist = float('inf')
            closest_link = None
            x_link = y_link = None
            for link in self.robot_links:
                try:
                    tf = self.tf_buffer.lookup_transform("laser", link, rclpy.time.Time())
                    lx = tf.transform.translation.x
                    ly = tf.transform.translation.y
                    dist = np.linalg.norm([x - lx, y - ly])
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_link = link
                        x_link = lx
                        y_link = ly
                except Exception:
                    continue

            speed = np.linalg.norm([vx, vy])
            radial_velocity = 0.0
            if x_link is not None:
                r_vec = np.array([x - x_link, y - y_link])
                if np.linalg.norm(r_vec) > 0:
                    radial_velocity = np.dot([vx, vy], r_vec) / np.linalg.norm(r_vec)

            self.get_logger().info(
                f"[Person {i+1}] Closest link: {closest_link}, Speed: {speed:.2f} m/s, Radial: {radial_velocity:.2f} m/s"
            )

        self.publisher_centroids.publish(marker_array)

    def cluster_dynamic_points(self, points):
        if not points:
            return [], []
            
        clusters = []  # List of lists containing points
        centroids = []  # List of cluster centroids
        used = [False] * len(points)
        
        # Start first cluster with first point
        first_point = np.array(points[0])
        clusters.append([points[0]])
        centroids.append(first_point)
        used[0] = True
        
        # For each remaining point
        for i in range(1, len(points)):
            if used[i]:
                continue
                
            current_point = np.array(points[i])
            min_dist = float('inf')
            closest_cluster_idx = -1
            
            # Find closest cluster centroid
            for j, centroid in enumerate(centroids):
                dist = np.linalg.norm(current_point - centroid)
                if dist < min_dist:
                    min_dist = dist
                    closest_cluster_idx = j
            
            # If close enough to an existing cluster, add to it and update centroid
            if min_dist < self.cluster_distance_threshold:
                clusters[closest_cluster_idx].append(points[i])
                # Update centroid as mean of all points in cluster
                centroids[closest_cluster_idx] = np.mean(clusters[closest_cluster_idx], axis=0)
                used[i] = True
            else:
                # Start new cluster
                clusters.append([points[i]])
                centroids.append(current_point)
                used[i] = True
        
        return clusters, centroids

    def clear_markers(self, publisher):
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        delete_all_marker.header.frame_id = "laser"
        delete_all_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_array.markers.append(delete_all_marker)
        publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundSubtractionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
