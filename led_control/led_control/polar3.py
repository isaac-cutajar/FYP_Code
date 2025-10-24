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

        # Define LED layout
        self.led_sequence = (
            list(range(0, 35)) +
            list(range(35, 57)) +
            list(range(164, 123, -1)) +
            list(range(125, 90, -1))
        )

        # Precompute angle-to-LED mapping (LED 47 = 0 deg, LED 1 = +90, LED 91 = -90)
        self.angle_to_led_map = self.generate_led_angle_map()

        self.sub = self.create_subscription(
            PoseArray,
            '/all_human_positions',
            self.pose_array_callback,
            10
        )

    def generate_led_angle_map(self):
        layout = self.led_sequence
        angle_led_map = {}
        total_leds = len(layout)

        for i, led_index in enumerate(layout):
            angle = 90.0 - (180.0 * i / (total_leds - 1))  # angle goes from +90 to -90
            angle_led_map[angle] = led_index

        return angle_led_map

    def angle_to_led(self, angle_deg):
        closest_angle = min(self.angle_to_led_map.keys(), key=lambda a: abs(a - angle_deg))
        return self.angle_to_led_map[closest_angle]

    def pose_array_callback(self, msg):
        active_leds = set()

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y

            angle_rad = math.atan2(x, y)  # x = lateral, y = forward
            angle_deg = math.degrees(angle_rad)
            angle_deg = max(-90.0, min(90.0, angle_deg))

            center_led = self.angle_to_led(angle_deg)
            center_idx = self.led_sequence.index(center_led)
            indices = [self.led_sequence[i % len(self.led_sequence)] for i in range(center_idx - 12, center_idx + 13)]
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
