#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QStatusBar, QFrame, QSizePolicy, QGridLayout)
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap, QColor, QPainter, QIcon
from PyQt5.QtWebEngineWidgets import QWebEngineView
try:
    from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
except ImportError:
    print("PyQt5.QtChart not found. Please install it with 'pip install PyQt5.QtChart'")
    sys.exit(1)
from sensor_msgs.msg import NavSatFix, Imu, Range, Image
from cv_bridge import CvBridge, CvBridgeError
import csv
import time
import os


class ROSWorker(QThread):
    ros_signal = pyqtSignal(str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.executor = MultiThreadedExecutor()
        self.running = True

    def run(self):
        self.executor.add_node(self.ros_node)
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            self.ros_signal.emit("ROS2 Node Running")
        self.executor.shutdown()

    def stop(self):
        self.running = False


class SensorGUI(QMainWindow):
    def __init__(self, ros_node, ros_worker):
        super().__init__()
        self.setWindowTitle("Sensor Data Dashboard")
        self.ros_node = ros_node
        self.ros_worker = ros_worker
        self.paused = False
        self.logging = False
        self.imu_x_data = []
        self.imu_y_data = []
        self.imu_z_data = []
        self.ultrasonic_data = []
        self.time_data = []
        self.csv_file = None
        self.csv_writer = None

        # Set larger window size
        self.resize(1200, 800)
        self.setStyleSheet("background-color: #34495e;")
        font = QFont("Helvetica", 12, QFont.Bold)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.ros_worker.ros_signal.connect(self.update_status)

        # Main widget and layout
        main_widget = QWidget()
        main_layout = QGridLayout()

        # Camera Section
        self.camera_frame = QFrame()
        self.camera_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        self.camera_frame.setStyleSheet("background-color: white;")
        camera_layout = QVBoxLayout()
        self.label_camera = QLabel("Camera Feed: Waiting...")
        self.label_camera.setAlignment(Qt.AlignCenter)
        self.label_camera.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_status = QLabel("●", self.label_camera)
        self.camera_status.setStyleSheet("color: gray; font-size: 20px;")
        self.camera_status.move(10, 10)
        camera_layout.addWidget(self.label_camera)
        self.save_camera_btn = QPushButton("Save Frame")
        self.save_camera_btn.setIcon(QIcon.fromTheme("document-save"))
        self.save_camera_btn.setStyleSheet("background-color: #2ecc71; color: white; padding: 6px;")
        self.save_camera_btn.clicked.connect(self.save_camera_frame)
        camera_layout.addWidget(self.save_camera_btn)
        self.camera_frame.setLayout(camera_layout)
        main_layout.addWidget(self.camera_frame, 0, 0, 2, 1)  # Span 2 rows, 1 column

        # Sensor Data Section
        sensor_frame = QFrame()
        sensor_frame.setFrameStyle(QFrame.Box | QFrame.Raised)
        sensor_frame.setStyleSheet("background-color: #ecf0f1;")
        sensor_layout = QVBoxLayout()

        self.label_gps = QLabel("GPS Data: Waiting...")
        self.label_gps.setFont(font)
        self.label_gps.setStyleSheet("color: #2c3e50; padding: 5px;")
        self.gps_status = QLabel("●", self.label_gps)
        self.gps_status.setStyleSheet("color: gray; font-size: 20px;")
        self.gps_status.move(10, 5)

        self.label_ultrasonic = QLabel("Ultrasonic Distance: Waiting...")
        self.label_ultrasonic.setFont(font)
        self.label_ultrasonic.setStyleSheet("color: #2c3e50; padding: 5px;")
        self.ultrasonic_status = QLabel("●", self.label_ultrasonic)
        self.ultrasonic_status.setStyleSheet("color: gray; font-size: 20px;")
        self.ultrasonic_status.move(10, 5)

        self.label_imu = QLabel("IMU Data: Waiting...")
        self.label_imu.setFont(font)
        self.label_imu.setStyleSheet("color: #2c3e50; padding: 5px;")
        self.imu_status = QLabel("●", self.label_imu)
        self.imu_status.setStyleSheet("color: gray; font-size: 20px;")
        self.imu_status.move(10, 5)

        sensor_layout.addWidget(self.label_gps)
        sensor_layout.addWidget(self.label_ultrasonic)
        sensor_layout.addWidget(self.label_imu)
        self.pause_button = QPushButton("Pause Updates")
        self.pause_button.setIcon(QIcon.fromTheme("media-playback-pause"))
        self.pause_button.setStyleSheet("background-color: #3498db; color: white; padding: 6px;")
        self.pause_button.clicked.connect(self.toggle_pause)
        sensor_layout.addWidget(self.pause_button)
        self.log_button = QPushButton("Start Logging")
        self.log_button.setIcon(QIcon.fromTheme("document-save"))
        self.log_button.setStyleSheet("background-color: #2ecc71; color: white; padding: 6px;")
        self.log_button.clicked.connect(self.toggle_logging)
        sensor_layout.addWidget(self.log_button)
        sensor_frame.setLayout(sensor_layout)
        main_layout.addWidget(sensor_frame, 0, 1, 1, 1)  # Top right

        # IMU Graph
        self.imu_chart = QChart()
        self.imu_chart.setTitle("IMU Orientation")
        self.imu_chart.legend().setVisible(True)
        self.x_series = QLineSeries()
        self.x_series.setName("X")
        self.y_series = QLineSeries()
        self.y_series.setName("Y")
        self.z_series = QLineSeries()
        self.z_series.setName("Z")
        self.imu_chart.addSeries(self.x_series)
        self.imu_chart.addSeries(self.y_series)
        self.imu_chart.addSeries(self.z_series)
        imu_axis_x = QValueAxis()
        imu_axis_x.setTitleText("Time (s)")
        imu_axis_x.setRange(0, 10)
        imu_axis_y = QValueAxis()
        imu_axis_y.setTitleText("Orientation")
        imu_axis_y.setRange(-1, 1)
        self.imu_chart.setAxisX(imu_axis_x, self.x_series)
        self.imu_chart.setAxisY(imu_axis_y, self.x_series)
        self.imu_chart.setAxisX(imu_axis_x, self.y_series)
        self.imu_chart.setAxisY(imu_axis_y, self.y_series)
        self.imu_chart.setAxisX(imu_axis_x, self.z_series)
        self.imu_chart.setAxisY(imu_axis_y, self.z_series)
        imu_chart_view = QChartView(self.imu_chart)
        imu_chart_view.setRenderHint(QPainter.Antialiasing)
        imu_chart_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        main_layout.addWidget(imu_chart_view, 1, 1, 1, 1)  # Bottom right

        # Ultrasonic Graph
        self.ultrasonic_chart = QChart()
        self.ultrasonic_chart.setTitle("Ultrasonic Distance")
        self.ultrasonic_series = QLineSeries()
        self.ultrasonic_series.setName("Distance (m)")
        self.ultrasonic_chart.addSeries(self.ultrasonic_series)
        ultra_axis_x = QValueAxis()
        ultra_axis_x.setTitleText("Time (s)")
        ultra_axis_x.setRange(0, 10)
        ultra_axis_y = QValueAxis()
        ultra_axis_y.setTitleText("Distance (m)")
        ultra_axis_y.setRange(0, 5)
        self.ultrasonic_chart.setAxisX(ultra_axis_x, self.ultrasonic_series)
        self.ultrasonic_chart.setAxisY(ultra_axis_y, self.ultrasonic_series)
        ultra_chart_view = QChartView(self.ultrasonic_chart)
        ultra_chart_view.setRenderHint(QPainter.Antialiasing)
        ultra_chart_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        main_layout.addWidget(ultra_chart_view, 0, 2, 1, 1)  # Top far right

        # GPS Map
        self.map_view = QWebEngineView()
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_view.setHtml("""
            <!DOCTYPE html>
            <html>
            <head>
                <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                <style>html, body, #map { height: 100%; margin: 0; }</style>
            </head>
            <body>
                <div id="map"></div>
                <script>
                    var map = L.map('map').setView([0, 0], 13);
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                        attribution: '&copy; OpenStreetMap contributors'
                    }).addTo(map);
                    var marker = L.marker([0, 0]).addTo(map);
                    function updateMarker(lat, lon) {
                        marker.setLatLng([lat, lon]);
                        map.setView([lat, lon], 13);
                    }
                </script>
            </body>
            </html>
        """)
        main_layout.addWidget(self.map_view, 1, 2, 1, 1)  # Bottom far right

        # Set stretch factors for responsiveness
        main_layout.setColumnStretch(0, 2)  # Camera gets more width
        main_layout.setColumnStretch(1, 1)
        main_layout.setColumnStretch(2, 1)
        main_layout.setRowStretch(0, 1)
        main_layout.setRowStretch(1, 1)

        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.setText("Resume Updates" if self.paused else "Pause Updates")
        self.pause_button.setIcon(QIcon.fromTheme("media-playback-start" if self.paused else "media-playback-pause"))
        self.pause_button.setStyleSheet(
            "background-color: #e74c3c; color: white; padding: 6px;" if self.paused 
            else "background-color: #3498db; color: white; padding: 6px;"
        )

    def toggle_logging(self):
        if not self.logging:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self.csv_file = open(f"sensor_log_{timestamp}.csv", 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(["Time", "GPS_Lat", "GPS_Lon", "GPS_Alt", "Ultrasonic", "IMU_X", "IMU_Y", "IMU_Z"])
            self.logging = True
            self.log_button.setText("Stop Logging")
            self.log_button.setStyleSheet("background-color: #e74c3c; color: white; padding: 6px;")
            self.status_bar.showMessage(f"Started logging to sensor_log_{timestamp}.csv", 3000)
        else:
            self.logging = False
            self.csv_file.close()
            self.log_button.setText("Start Logging")
            self.log_button.setStyleSheet("background-color: #2ecc71; color: white; padding: 6px;")
            self.status_bar.showMessage("Stopped logging", 3000)

    def update_gui(self):
        if self.paused:
            return
        try:
            current_time = time.time() - self.start_time if hasattr(self, 'start_time') else 0
            if not hasattr(self, 'start_time'):
                self.start_time = time.time()
            self.time_data.append(current_time)
            if len(self.time_data) > 100:
                self.time_data.pop(0)

            if self.ros_node.gps_data is not None:
                self.label_gps.setText(
                    f"GPS: Lat: {self.ros_node.gps_data.latitude:.4f}, "
                    f"Lon: {self.ros_node.gps_data.longitude:.4f}, "
                    f"Alt: {self.ros_node.gps_data.altitude:.2f}"
                )
                self.gps_status.setStyleSheet("color: green; font-size: 20px;")
                self.map_view.page().runJavaScript(
                    f"updateMarker({self.ros_node.gps_data.latitude}, {self.ros_node.gps_data.longitude});"
                )

            if self.ros_node.ultrasonic_data is not None:
                self.label_ultrasonic.setText(
                    f"Ultrasonic: {self.ros_node.ultrasonic_data:.2f} meters"
                )
                self.ultrasonic_status.setStyleSheet("color: green; font-size: 20px;")
                self.ultrasonic_data.append(self.ros_node.ultrasonic_data)
                if len(self.ultrasonic_data) > 100:
                    self.ultrasonic_data.pop(0)
                self.ultrasonic_series.clear()
                for t, dist in zip(self.time_data, self.ultrasonic_data):
                    self.ultrasonic_series.append(t, dist)
                self.ultrasonic_chart.axisX().setRange(max(0, current_time - 10), current_time)

            if self.ros_node.imu_data is not None:
                self.label_imu.setText(
                    f"IMU: x={self.ros_node.imu_data.orientation.x:.2f}, "
                    f"y={self.ros_node.imu_data.orientation.y:.2f}, "
                    f"z={self.ros_node.imu_data.orientation.z:.2f}"
                )
                self.imu_status.setStyleSheet("color: green; font-size: 20px;")
                self.imu_x_data.append(self.ros_node.imu_data.orientation.x)
                self.imu_y_data.append(self.ros_node.imu_data.orientation.y)
                self.imu_z_data.append(self.ros_node.imu_data.orientation.z)
                if len(self.imu_x_data) > 100:
                    self.imu_x_data.pop(0)
                    self.imu_y_data.pop(0)
                    self.imu_z_data.pop(0)
                self.x_series.clear()
                self.y_series.clear()
                self.z_series.clear()
                for t, x, y, z in zip(self.time_data, self.imu_x_data, self.imu_y_data, self.imu_z_data):
                    self.x_series.append(t, x)
                    self.y_series.append(t, y)
                    self.z_series.append(t, z)
                self.imu_chart.axisX().setRange(max(0, current_time - 10), current_time)

            if self.ros_node.camera_data is not None:
                height, width, channel = self.ros_node.camera_data.shape
                bytes_per_line = 3 * width
                q_img = QImage(self.ros_node.camera_data.tobytes(), width, height, bytes_per_line, QImage.Format_BGR888)
                pixmap = QPixmap.fromImage(q_img).scaled(self.label_camera.size(), aspectRatioMode=1)
                self.label_camera.setPixmap(pixmap)
                self.camera_status.setStyleSheet("color: green; font-size: 20px;")

            if self.logging:
                self.csv_writer.writerow([
                    current_time,
                    self.ros_node.gps_data.latitude if self.ros_node.gps_data else None,
                    self.ros_node.gps_data.longitude if self.ros_node.gps_data else None,
                    self.ros_node.gps_data.altitude if self.ros_node.gps_data else None,
                    self.ros_node.ultrasonic_data if self.ros_node.ultrasonic_data else None,
                    self.ros_node.imu_data.orientation.x if self.ros_node.imu_data else None,
                    self.ros_node.imu_data.orientation.y if self.ros_node.imu_data else None,
                    self.ros_node.imu_data.orientation.z if self.ros_node.imu_data else None
                ])
        except Exception as e:
            self.status_bar.showMessage(f"Error: {e}", 5000)

    def save_camera_frame(self):
        if self.ros_node.camera_data is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"camera_frame_{timestamp}.png", self.ros_node.camera_data)
            self.status_bar.showMessage(f"Saved camera frame as camera_frame_{timestamp}.png", 3000)

    def update_status(self, message):
        self.status_bar.showMessage(message)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.ros_node.camera_data is not None:
            self.update_gui()

    def closeEvent(self, event):
        if self.logging:
            self.csv_file.close()
        self.ros_worker.stop()
        self.ros_worker.wait()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_gui')
        self.gps_data = None
        self.ultrasonic_data = None
        self.imu_data = None
        self.camera_data = None
        self.bridge = CvBridge()
        self.create_subscription(NavSatFix, 'gps_data', self.update_gps, 10)
        self.create_subscription(Range, 'ultrasonic_data', self.update_ultrasonic, 10)
        self.create_subscription(Imu, 'imu_data', self.update_imu, 10)
        self.create_subscription(Image, 'camera/image_raw', self.update_camera, 10)

    def update_gps(self, msg):
        self.gps_data = msg

    def update_ultrasonic(self, msg):
        self.ultrasonic_data = msg.range

    def update_imu(self, msg):
        self.imu_data = msg

    def update_camera(self, msg):
        try:
            self.camera_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main(args=None):
    rclpy.init(args=args)
    ros_node = SensorNode()
    ros_worker = ROSWorker(ros_node)
    ros_worker.start()
    app = QApplication(sys.argv)
    gui = SensorGUI(ros_node, ros_worker)
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()






