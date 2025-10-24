import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32MultiArray
import serial
import time

class SpeedLedFeedbackNode(Node):
    def __init__(self):
        super().__init__('speed_led_feedback_node')

        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)

        self.led_numbers = []
        self.current_color = (255, 0, 0)

        self.led_sub = self.create_subscription(
            Int32MultiArray,
            '/led_number',
            self.led_callback,
            10
        )

        self.speed_sub = self.create_subscription(
            Float64,
            '/speed_scale_fraction',
            self.speed_callback,
            10
        )

    def led_callback(self, msg):
        self.led_numbers = msg.data
        self.get_logger().info(f"LED indices: {self.led_numbers}")
        self.send_command()

    def speed_callback(self, msg):
        value = float(msg.data)
        value = max(0.1, min(1.0, value))  # clamp to [0.1, 1.0]

        scale = (value - 0.1) / 0.9  # normalize to 0.0–1.0

        r = int(255 * (1.0 - scale))
        g = int(255 * scale)
        b = 0

        self.current_color = (r, g, b)
        self.get_logger().info(f"Speed scale: {value:.2f}, Color: R{r} G{g} B{b}")
        self.send_command()

    def send_command(self):
        if not self.led_numbers:
            return

        r, g, b = self.current_color
        command = "B," + ";".join(
            [f"{i},{r},{g},{b}" for i in self.led_numbers]
        ) + "\n"

        self.get_logger().info(f"Sending to Arduino: {command.strip()}")
        self.ser.write(command.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SpeedLedFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
