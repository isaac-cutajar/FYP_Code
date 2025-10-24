#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QGroupBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class ROS2StatusGUI(Node):
    def __init__(self):
        super().__init__('ros2_status_gui')

        # Value labels
        self.distance_label = QLabel("N/A")
        self.speed_scale_label = QLabel("N/A")
        self.sensor_status_label = QLabel("N/A")
        self.threshold_status_label = QLabel("N/A")
        self.protective_status_label = QLabel("N/A")

        # Font for values
        value_font = QFont()
        value_font.setPointSize(32)
        value_font.setBold(True)

        for label in [
            self.distance_label,
            self.speed_scale_label,
            self.sensor_status_label,
            self.threshold_status_label,
            self.protective_status_label
        ]:
            label.setFont(value_font)
            label.setAlignment(Qt.AlignCenter)

        # ROS topic subscriptions
        self.create_subscription(String, '/shortest_distance', self.update_distance, 10)
        self.create_subscription(Float64, '/speed_scale_fraction', self.update_speed_scale, 10)
        self.create_subscription(Int32, '/sensor_status', self.update_sensor_status, 10)
        self.create_subscription(Int32, '/detection_consistency', self.update_threshold_status, 10)
        self.create_subscription(Int32, '/safety_status', self.update_protective_status, 10)

    def update_distance(self, msg):
        self.distance_label.setText(msg.data)

    def update_speed_scale(self, msg):
        value = msg.data
        self.speed_scale_label.setText(f"{value:.2f}")
        self.set_speed_color(value)

    def set_speed_color(self, value):
        # Clamp between 0.1 and 1.0
        clamped = max(0.1, min(value, 1.0))
        t = (clamped - 0.1) / (1.0 - 0.1)  # Normalize to 0-1

        # Linear interpolation: Red to Green
        r = int((1.0 - t) * 255)
        g = int(t * 255)
        b = 0

        color = f"rgb({r},{g},{b})"
        self.speed_scale_label.setStyleSheet(f"color: {color};")

    def update_sensor_status(self, msg):
        self.sensor_status_label.setText(self.status_str(msg.data))

    def update_threshold_status(self, msg):
        self.threshold_status_label.setText(self.status_str(msg.data))

    def update_protective_status(self, msg):
        self.protective_status_label.setText(self.status_str(msg.data))

    def status_str(self, value):
        return "✅ OK" if value == 1 else "❌ ERROR"


class GUIWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Robot Status Panel")
        self.setGeometry(900, 100, 400, 600)

        # Font for headers
        header_font = QFont()
        header_font.setPointSize(16)
        header_font.setBold(False)

        layout = QVBoxLayout()

        # Top section
        top_box = QGroupBox("Robot Proximity Info")
        top_layout = QVBoxLayout()

        label1 = QLabel("Distance to Closest Robot Link:")
        label1.setFont(header_font)
        top_layout.addWidget(label1)
        top_layout.addWidget(ros_node.distance_label)

        label2 = QLabel("Speed Scale Fraction:")
        label2.setFont(header_font)
        top_layout.addWidget(label2)
        top_layout.addWidget(ros_node.speed_scale_label)

        top_layout.setSpacing(20)
        top_box.setLayout(top_layout)

        # Bottom section
        bottom_box = QGroupBox("Safety Nodes Status")
        bottom_layout = QVBoxLayout()

        label3 = QLabel("Sensor Status:")
        label3.setFont(header_font)
        bottom_layout.addWidget(label3)
        bottom_layout.addWidget(ros_node.sensor_status_label)

        label4 = QLabel("Threshold Status:")
        label4.setFont(header_font)
        bottom_layout.addWidget(label4)
        bottom_layout.addWidget(ros_node.threshold_status_label)

        label5 = QLabel("Protective Separation Status:")
        label5.setFont(header_font)
        bottom_layout.addWidget(label5)
        bottom_layout.addWidget(ros_node.protective_status_label)

        bottom_layout.setSpacing(20)
        bottom_box.setLayout(bottom_layout)

        layout.addWidget(top_box)
        layout.addWidget(bottom_box)
        layout.setSpacing(30)
        self.setLayout(layout)


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    node = ROS2StatusGUI()
    win = GUIWindow(node)
    win.show()

    # Create a QTimer to spin ROS2 in the main thread
    from PyQt5.QtCore import QTimer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)  # spin every 10ms

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

