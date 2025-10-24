import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import serial
import time
import math

class LidarToLEDNode(Node):
    def __init__(self):
        super().__init__('lidar_to_led_node')

        # Setup serial to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)

        # Full physical LED sequence (U shape)
        self.led_sequence = list(range(1, 57)) + list(range(164, 90, -1))

        # Key angle anchors (user provided)
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

        # Interpolated map: LED number → angle
        self.led_angle_map = self.interpolate_led_angles(self.led_sequence, anchor_angles)

        self.sub = self.create_subscription(
            PoseArray,
            '/all_human_positions',
            self.pose_array_callback,
            10
        )

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

    def pose_array_callback(self, msg):
        active_leds = set()

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

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
            active_leds.update(indices)

        if active_leds:
            r, g, b = 255, 0, 0
            command = "B," + ";".join(f"{i},{r},{g},{b}" for i in sorted(active_leds)) + "\n"
            self.ser.write(command.encode())
            print(f"[POSEARRAY] Active LEDs: {sorted(active_leds)}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarToLEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
