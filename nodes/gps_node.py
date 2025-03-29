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

        # Attempt to open the serial port
        try:
            self.ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Adjust port if necessary
            self.get_logger().info("GPS Node Started on /dev/serial0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPS module. Check wiring! Exception: {e}")
            self.ser = None  # Ensure self.ser is defined

    def read_gps(self):
        # This debug statement prints every time the loop executes.
        self.get_logger().debug("Loop executed")

        # Verify that the serial connection exists
        if self.ser is None:
            return

        # Check if data is available
        if self.ser.in_waiting > 0:
            try:
                # Read and decode the incoming line
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().debug(f"Raw line received: {line}")  # Debug logging

                # Process only $GPGGA sentences
                if line.startswith('$GPGGA'):
                    data = pynmea2.parse(line)
                    latitude = self.convert_to_decimal(data.latitude, data.lat_dir)
                    longitude = self.convert_to_decimal(data.longitude, data.lon_dir)

                    msg = NavSatFix()
                    msg.latitude = latitude
                    msg.longitude = longitude
                    msg.altitude = float(data.altitude) if data.altitude else 0.0
                    
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Published GPS Data: Lat {latitude}, Lon {longitude}, Alt {msg.altitude}")
            except (pynmea2.ParseError, ValueError) as e:
                self.get_logger().warn(f"GPS Parsing Error: {e}")

    def convert_to_decimal(self, value, direction):
        """
        Convert a coordinate from degrees and minutes (DDMM.MMMM) to decimal degrees (DD.DDDDDD)
        """
        if not value:
            return 0.0
        degrees = int(float(value) / 100)
        minutes = float(value) - (degrees * 100)
        decimal = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

    def destroy_node(self):
        if self.ser is not None:
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
