#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2  # Library to parse NMEA sentences

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        print("Initialized Node")
        self.publisher = self.create_publisher(NavSatFix, 'gps_data', 10)
        print("Publisher created")
        self.timer = self.create_timer(1.0, self.read_gps)
        print("Timer created with 1.0 second interval")
        
        # Attempt to open the serial port
        try:
            self.ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Adjust port if necessary
            self.get_logger().info("GPS Node Started on /dev/serial0")
            print("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPS module. Check wiring! Exception: {e}")
            print("Serial port open failed")
            self.ser = None  # Ensure self.ser is defined
            print("self.ser set to None")

    def read_gps(self):
        # Print a message every time the loop executes.
        print("read_gps() called")
        self.get_logger().debug("Loop executed")
        
        # Verify that the serial connection exists
        if self.ser is None:
            print("No serial connection; returning from read_gps()")
            return
        else:
            print("Serial connection exists")

        # Check if data is available
        if self.ser.in_waiting > 0:
            print("Data available on serial port")
            try:
                # Read and decode the incoming line
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                print("Read line from serial port")
                self.get_logger().debug(f"Raw line received: {line}")  # Debug logging

                # Process only $GPGGA sentences
                if line.startswith('$GPGGA'):
                    print("Line starts with $GPGGA")
                    data = pynmea2.parse(line)
                    print("Parsed NMEA sentence")
                    latitude = self.convert_to_decimal(data.latitude, data.lat_dir)
                    print("Converted latitude to decimal")
                    longitude = self.convert_to_decimal(data.longitude, data.lon_dir)
                    print("Converted longitude to decimal")

                    msg = NavSatFix()
                    print("Created NavSatFix message")
                    msg.latitude = latitude
                    print("Set message latitude")
                    msg.longitude = longitude
                    print("Set message longitude")
                    msg.altitude = float(data.altitude) if data.altitude else 0.0
                    print("Set message altitude")

                    self.publisher.publish(msg)
                    print("Published GPS data")
                    self.get_logger().info(f"Published GPS Data: Lat {latitude}, Lon {longitude}, Alt {msg.altitude}")
                else:
                    print("Line does not start with $GPGGA; ignoring")
            except (pynmea2.ParseError, ValueError) as e:
                self.get_logger().warn(f"GPS Parsing Error: {e}")
                print("Caught parsing error in read_gps()")
        else:
            print("No data available on serial port")

    def convert_to_decimal(self, value, direction):
        """
        Convert a coordinate from degrees and minutes (DDMM.MMMM) to decimal degrees (DD.DDDDDD)
        """
        print(f"Converting value: {value} with direction: {direction}")
        if not value:
            print("No value provided; returning 0.0")
            return 0.0
        degrees = int(float(value) / 100)
        print(f"Extracted degrees: {degrees}")
        minutes = float(value) - (degrees * 100)
        print(f"Extracted minutes: {minutes}")
        decimal = degrees + (minutes / 60)
        print(f"Calculated decimal before direction check: {decimal}")
        if direction in ['S', 'W']:
            decimal = -decimal
            print("Direction is South or West; negated decimal value")
        print(f"Final decimal value: {decimal}")
        return decimal

    def destroy_node(self):
        print("Destroying node")
        if self.ser is not None:
            self.ser.close()
            print("Serial port closed")
        super().destroy_node()
        print("Node destroyed")

def main(args=None):
    print("Initializing rclpy")
    rclpy.init(args=args)
    print("Creating GPSNode instance")
    node = GPSNode()
    print("Spinning node")
    rclpy.spin(node)
    print("Spin complete, destroying node")
    node.destroy_node()
    rclpy.shutdown()
    print("Shutdown complete")

if __name__ == '__main__':
    main()
