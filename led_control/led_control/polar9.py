import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import serial
import time
import math

class LidarToLEDNode(Node):
    def __init__(self):
        super().__init__('lidar_to_led_node')

        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        time.sleep(2)

        self.led_sequence = list(range(1, 57)) + list(range(164, 90, -1))
        self.full_led_range = list(range(0, 165))

        anchor_angles = {
            1: 90.0,
            10: 75.0,
            20: 45.0,
            35: 0.0,
            56: -30.0,
            155: -45.0,
            120: -60.0,
            91: -90.0
        }

        self.led_angle_map = self.interpolate_led_angles(self.led_sequence, anchor_angles)

        self.sensor_status = True
        self.safety_status = True
        self.detection_consistency = True

        self.create_subscription(PoseArray, '/all_human_positions', self.pose_array_callback, 10)
        self.create_subscription(Int32, '/sensor_status', self.sensor_status_callback, 10)
        self.create_subscription(Int32, '/safety_status', self.safety_status_callback, 10)
        self.create_subscription(Int32, '/detection_consistency', self.detection_consistency_callback, 10)

    def interpolate_led_angles(self, led_sequence, anchors):
        full_map = {}
        sorted_anchors = sorted(anchors.items())
        anchor_leds = [a[0] for a in sorted_anchors]
        anchor_angles = [a[1] for a in sorted_anchors]

        for i in range(len(anchor_leds) - 1):
            start_led = anchor_leds[i]
            end_led = anchor_leds[i + 1]
            start_angle = anchor_angles[i]
            end_angle = anchor_angles[i + 1]

            if start_led not in led_sequence or end_led not in led_sequence:
                continue

            start_idx = led_sequence.index(start_led)
            end_idx = led_sequence.index(end_led)

            if start_idx > end_idx:
                start_led, end_led = end_led, start_led
                start_angle, end_angle = end_angle, start_angle
                start_idx, end_idx = end_idx, start_idx

            steps = end_idx - start_idx
            for j in range(steps + 1):
                led = led_sequence[start_idx + j]
                angle = start_angle + (end_angle - start_angle) * j / steps
                full_map[led] = angle

        return full_map

    def angle_to_led(self, angle_deg):
        closest_led = min(
            self.led_angle_map.keys(),
            key=lambda led: abs(self.led_angle_map[led] - angle_deg)
        )
        if abs(self.led_angle_map[closest_led] - angle_deg) > 30:
            return None
        return closest_led

    def sensor_status_callback(self, msg):
        self.sensor_status = msg.data

    def safety_status_callback(self, msg):
        self.safety_status = msg.data

    def detection_consistency_callback(self, msg):
        self.detection_consistency = msg.data

    def pose_array_callback(self, msg):
        # Check override conditions first
        if not (self.sensor_status and self.safety_status and self.detection_consistency):
            command = "B," + ";".join(f"{i},255,0,0" for i in self.full_led_range) + "\n"
            self.ser.write(command.encode())
            print("[STATUS] ERROR CONDITION: All LEDs RED")
            return

        if len(msg.poses) == 0:
            command = "B," + ";".join(f"{i},0,255,0" for i in self.full_led_range) + "\n"
            self.ser.write(command.encode())
            print("[STATUS] No humans: All LEDs GREEN")
            return

        led_color_map = {}

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            distance = max(0.0, math.sqrt(x**2 + y**2) - 0.5)

            if distance <= 0.1:
                r, g, b = 255, 0, 0
            elif distance >= 2.0:
                r, g, b = 0, 255, 0
            else:
                ratio = (distance - 0.1) / (2.0 - 0.1)
                r = int(255 * (1 - ratio))
                g = int(255 * ratio)
                b = 0

            angle_rad = math.atan2(x, y)
            angle_deg = math.degrees(angle_rad)
            angle_deg = max(-90.0, min(90.0, angle_deg))

            center_led = self.angle_to_led(angle_deg)
            if center_led is None:
                continue

            center_idx = self.led_sequence.index(center_led)
            indices = [
                self.led_sequence[i]
                for i in range(center_idx - 12, center_idx + 13)
                if 0 <= i < len(self.led_sequence)
            ]

            for i in indices:
                led_color_map[i] = (r, g, b)

        if led_color_map:
            command = "B," + ";".join(f"{i},{r},{g},{b}" for i, (r, g, b) in sorted(led_color_map.items())) + "\n"
            self.ser.write(command.encode())
            print(f"[POSEARRAY] Sent {len(led_color_map)} LEDs")


def main(args=None):
    rclpy.init(args=args)
    node = LidarToLEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

