#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QGroupBox
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QFont
import re


class ROS2Signals(QObject):
    distance_signal = pyqtSignal(str)
    speed_scale_signal = pyqtSignal(float)
    sensor_status_signal = pyqtSignal(int)
    threshold_status_signal = pyqtSignal(int)
    protective_status_signal = pyqtSignal(int)


class ROS2StatusGUI(Node):
    def __init__(self, signals):
        super().__init__('ros2_status_gui')
        self.signals = signals

        self.create_subscription(String, '/shortest_distance', self.update_distance, 10)
        self.create_subscription(Float64, '/speed_scale_fraction', self.update_speed_scale, 10)
        self.create_subscription(Int32, '/sensor_status', self.update_sensor_status, 10)
        self.create_subscription(Int32, '/detection_consistency', self.update_threshold_status, 10)
        self.create_subscription(Int32, '/safety_status', self.update_protective_status, 10)

    def update_distance(self, msg):
        try:
            match = re.search(r"([-+]?\d*\.\d+|\d+)", msg.data)
            if match:
                distance = float(match.group(0))
                self.signals.distance_signal.emit(f"{distance:.2f} m")
            else:
                self.signals.distance_signal.emit("No person detected")
        except Exception:
            self.signals.distance_signal.emit("Invalid value")

    def update_speed_scale(self, msg):
        try:
            value = float(msg.data)
            self.signals.speed_scale_signal.emit(value)
        except Exception:
            pass

    def update_sensor_status(self, msg):
        self.signals.sensor_status_signal.emit(msg.data)

    def update_threshold_status(self, msg):
        self.signals.threshold_status_signal.emit(msg.data)

    def update_protective_status(self, msg):
        self.signals.protective_status_signal.emit(msg.data)


class GUIWindow(QWidget):
    def __init__(self, signals):
        super().__init__()
        self.setWindowTitle("Robot Status Panel")
        self.setGeometry(900, 100, 400, 600)

        self.distance_label = QLabel("N/A")
        self.speed_scale_label = QLabel("N/A")
        self.sensor_status_label = QLabel("✅ OK")
        self.threshold_status_label = QLabel("✅ OK")
        self.protective_status_label = QLabel("✅ OK")

        # Fonts
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

        header_font = QFont()
        header_font.setPointSize(16)
        header_font.setBold(False)

        # Layouts
        layout = QVBoxLayout()

        # Proximity Info
        top_box = QGroupBox("Robot Proximity Info")
        top_layout = QVBoxLayout()
        label1 = QLabel("Distance to Closest Robot Link:")
        label1.setFont(header_font)
        top_layout.addWidget(label1)
        top_layout.addWidget(self.distance_label)

        label2 = QLabel("Speed Scale Fraction:")
        label2.setFont(header_font)
        top_layout.addWidget(label2)
        top_layout.addWidget(self.speed_scale_label)

        top_layout.setSpacing(20)
        top_box.setLayout(top_layout)

        # Safety Info
        bottom_box = QGroupBox("Safety Nodes Status")
        bottom_layout = QVBoxLayout()

        label3 = QLabel("Sensor Status:")
        label3.setFont(header_font)
        bottom_layout.addWidget(label3)
        bottom_layout.addWidget(self.sensor_status_label)

        label4 = QLabel("Threshold Status:")
        label4.setFont(header_font)
        bottom_layout.addWidget(label4)
        bottom_layout.addWidget(self.threshold_status_label)

        label5 = QLabel("Protective Separation Status:")
        label5.setFont(header_font)
        bottom_layout.addWidget(label5)
        bottom_layout.addWidget(self.protective_status_label)

        bottom_layout.setSpacing(20)
        bottom_box.setLayout(bottom_layout)

        layout.addWidget(top_box)
        layout.addWidget(bottom_box)
        layout.setSpacing(30)
        self.setLayout(layout)

        # Connect signals
        signals.distance_signal.connect(self.distance_label.setText)
        signals.speed_scale_signal.connect(self.set_speed_scale_text)
        signals.sensor_status_signal.connect(lambda val: self.sensor_status_label.setText(self.status_str(val)))
        signals.threshold_status_signal.connect(lambda val: self.threshold_status_label.setText(self.status_str(val)))
        signals.protective_status_signal.connect(lambda val: self.protective_status_label.setText(self.status_str(val)))

    def set_speed_scale_text(self, value):
        self.speed_scale_label.setText(f"{value:.2f}")
        self.set_speed_color(value)

    def set_speed_color(self, value):
        clamped = max(0.1, min(value, 1.0))
        t = (clamped - 0.1) / (1.0 - 0.1)
        r = int((1.0 - t) * 255)
        g = int(t * 255)
        b = 0
        self.speed_scale_label.setStyleSheet(f"color: rgb({r},{g},{b});")

    def status_str(self, value):
        return "✅ OK" if value == 1 else "❌ ERROR"


def main():
    rclpy.init()
    app = QApplication(sys.argv)

    signals = ROS2Signals()
    node = ROS2StatusGUI(signals)
    win = GUIWindow(signals)
    win.show()

    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
