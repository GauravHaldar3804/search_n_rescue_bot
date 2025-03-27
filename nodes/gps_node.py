#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2  # Library to parse NMEA sentences

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.publisher = self.create_publisher(NavSatFix, 'gps_data', 10)
        self.timer = self.create_timer(1.0, self.read_gps)

        # Open UART connection (adjust port based on your setup)
        try:
            self.ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Use /dev/ttyS0 if needed
            self.get_logger().info("GPS Node Started on /dev/serial0")
        except serial.SerialException:
            self.get_logger().error("Failed to connect to GPS module. Check wiring!")

    def read_gps(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('$GPGGA'):
                    data = pynmea2.parse(line)
                    latitude = self.convert_to_decimal(data.latitude, data.lat_dir)
                    longitude = self.convert_to_decimal(data.longitude, data.lon_dir)

                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude
                    msg.altitude = data.altitude if data.altitude else 0.0  # Add altitude if available
                    
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Published GPS Data: Lat {latitude}, Lon {longitude}, Alt {msg.altitude}")
            except (pynmea2.ParseError, ValueError) as e:
                self.get_logger().warn(f"GPS Parsing Error: {e}")

    def convert_to_decimal(self, value, direction):
        """Convert latitude/longitude from degrees and minutes (DDMM.MMMM) to decimal degrees (DD.DDDDDD)"""
        if not value:
            return 0.0
        degrees = int(float(value) / 100)
        minutes = float(value) - (degrees * 100)
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
