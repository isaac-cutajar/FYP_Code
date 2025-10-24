import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, ColorRGBA
import serial
import time

class LEDSerialNode(Node):
    def __init__(self):
        super().__init__('led_serial_node')

        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(2)  # Wait for Arduino to reset

        self.current_color = (255, 255, 255)
        self.led_numbers = []

        self.color_sub = self.create_subscription(
            ColorRGBA,
            '/color',
            self.color_callback,
            10
        )

        self.led_sub = self.create_subscription(
            Int32MultiArray,
            '/led_number',
            self.led_callback,
            10
        )

    def color_callback(self, msg):
        self.current_color = (int(msg.r), int(msg.g), int(msg.b))
        self.get_logger().info(f"Received color: {self.current_color}")
        self.send_command()

    def led_callback(self, msg):
        self.led_numbers = msg.data
        self.get_logger().info(f"Received LED indices: {self.led_numbers}")
        self.send_command()

    def send_command(self):
        if not self.led_numbers:
            return

        r, g, b = self.current_color
        command = "B," + ";".join(
            [f"{index},{r},{g},{b}" for index in self.led_numbers]
        ) + "\n"

        self.get_logger().info(f"Sending to Arduino: {command.strip()}")
        self.ser.write(command.encode())


def main(args=None):
    rclpy.init(args=args)
    node = LEDSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
