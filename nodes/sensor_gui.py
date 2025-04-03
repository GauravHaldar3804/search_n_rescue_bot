#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont
from sensor_msgs.msg import NavSatFix, Imu, Range
from std_msgs.msg import Float32


class ROSWorker(QThread):
    ros_signal = pyqtSignal()

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

    def run(self):
        """Run ROS2 node in a QThread."""
        rclpy.spin(self.ros_node)


class SensorGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Sensor Data GUI")

        # Set window size and font size
        self.resize(500, 350)  # Larger window size
        font = QFont()
        font.setPointSize(12)  # Larger font for readability

        # ROS Node instance passed from main
        self.ros_node = ros_node

        # Create QLabel widgets for displaying sensor data with larger font
        self.label_gps = QLabel("GPS Data: Waiting...")
        self.label_gps.setFont(font)

        self.label_ultrasonic = QLabel("Ultrasonic Distance: Waiting...")
        self.label_ultrasonic.setFont(font)

        self.label_imu = QLabel("IMU Data: Waiting...")
        self.label_imu.setFont(font)

        # Set up layout and add widgets
        layout = QVBoxLayout()
        layout.addWidget(self.label_gps)
        layout.addWidget(self.label_ultrasonic)
        layout.addWidget(self.label_imu)

        # Create a container widget and set layout
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Start a timer to periodically update the GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # Update every 100ms

        # Start ROS2 in a QThread
        self.ros_thread = ROSWorker(self.ros_node)
        self.ros_thread.start()

    def update_gui(self):
        """Update the GUI with the latest sensor data."""
        if self.ros_node.gps_data is not None:
            self.label_gps.setText(
                f"GPS Data: Lat: {self.ros_node.gps_data.latitude:.4f}, "
                f"Lon: {self.ros_node.gps_data.longitude:.4f}, "
                f"Alt: {self.ros_node.gps_data.altitude:.2f}"
            )

        if self.ros_node.ultrasonic_data is not None:
            self.label_ultrasonic.setText(
                f"Ultrasonic Distance: {self.ros_node.ultrasonic_data:.2f} meters"
            )

        if self.ros_node.imu_data is not None:
            self.label_imu.setText(
                f"IMU Orientation: x={self.ros_node.imu_data.orientation.x:.2f}, "
                f"y={self.ros_node.imu_data.orientation.y:.2f}, "
                f"z={self.ros_node.imu_data.orientation.z:.2f}"
            )


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_gui')
        self.gps_data = None
        self.ultrasonic_data = None
        self.imu_data = None

        # Create ROS2 subscriptions
        self.create_subscription(NavSatFix, 'gps_data', self.update_gps, 10)
        self.create_subscription(Range, 'ultrasonic_data', self.update_ultrasonic, 10)
        self.create_subscription(Imu, 'imu_data', self.update_imu, 10)

    def update_gps(self, msg):
        self.gps_data = msg

    def update_ultrasonic(self, msg):
        self.ultrasonic_data = msg.range

    def update_imu(self, msg):
        self.imu_data = msg


def main(args=None):
    rclpy.init(args=args)
    ros_node = SensorNode()

    # Run PyQt5 GUI in the main thread
    app = QApplication(sys.argv)
    gui = SensorGUI(ros_node)
    gui.show()
    sys.exit(app.exec_())

    # Shutdown ROS2
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





