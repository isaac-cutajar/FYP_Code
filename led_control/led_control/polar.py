import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial
import time
import math

class LidarToLEDNode(Node):
    def __init__(self):
        super().__init__('lidar_to_led_node')

        # Setup serial to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)

        # Precompute angle-to-LED mapping based on user's custom layout
        self.angle_to_led_map = self.generate_led_angle_map()

        self.sub = self.create_subscription(
            Point,
            '/lidar_human_position',
            self.lidar_callback,
            10
        )

    def generate_led_angle_map(self):
        # Create a mapping from angle (-90 to +90) to LED index based on custom U layout
        # We will interpolate across the physical layout

        layout = (
            list(range(0, 35)) +                     # First vertical: 0 to 34
            list(range(35, 57)) +                    # First horizontal: 35 to 56
            list(range(164, 123, -1)) +              # Second horizontal: 164 to 124
            list(range(125, 90, -1))                 # Second vertical: 125 to 91
        )

        angle_led_map = {}
        total_leds = len(layout)
        for i, led_index in enumerate(layout):
            angle = -90.0 + (180.0 * i / (total_leds - 1))  # Linear angle from -90 to +90
            angle_led_map[angle] = led_index
        return angle_led_map

    def angle_to_led(self, angle_deg):
        # Find closest angle in the map
        closest_angle = min(self.angle_to_led_map.keys(), key=lambda a: abs(a - angle_deg))
        return self.angle_to_led_map[closest_angle]

    def lidar_callback(self, msg):
        x, y = msg.x, msg.y
        angle_rad = math.atan2(y, x)
        angle_deg = math.degrees(angle_rad)
        angle_deg = max(-90.0, min(90.0, angle_deg))

        center_led = self.angle_to_led(angle_deg)

        # Flatten angle map to list for wraparound-friendly indexing
        led_sequence = (
            list(range(0, 35)) +
            list(range(35, 57)) +
            list(range(164, 123, -1)) +
            list(range(125, 90, -1))
        )

        center_idx = led_sequence.index(center_led)
        indices = [led_sequence[i % len(led_sequence)] for i in range(center_idx - 12, center_idx + 13)]

        r, g, b = 255, 0, 0  # You can change this logic to use distance-based color, etc.
        command = "B," + ";".join(f"{i},{r},{g},{b}" for i in indices) + "\n"
        self.ser.write(command.encode())


def main(args=None):
    rclpy.init(args=args)
    node = LidarToLEDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
