#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

# Define pins
TRIG = 23
ECHO = 24

class UltrasonicSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')
        self.publisher = self.create_publisher(Range, 'ultrasonic_data', 10)
        self.timer = self.create_timer(0.5, self.read_distance)

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)
        self.get_logger().info("Ultrasonic Sensor Node Started!")

    def read_distance(self):
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        start_time = time.time()
        stop_time = time.time()

        while GPIO.input(ECHO) == 0:
            start_time = time.time()

        while GPIO.input(ECHO) == 1:
            stop_time = time.time()

        # Compute distance
        distance = (stop_time - start_time) * 34300 / 2
        msg = Range()
        msg.range = distance
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Distance: {distance:.2f} cm')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
