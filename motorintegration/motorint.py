#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

# Import libraries to control GPIO and PCA9685 for PWM
import RPi.GPIO as GPIO
import board
import busio
from adafruit_pca9685 import PCA9685

class MotorTestNode(Node):
    def __init__(self):
        super().__init__('motor_test_node')

        # --- GPIO Setup for BTS7960 Enable Pins ---
        self.left_enable_pin = 17   # GPIO pin for left motor enable
        self.right_enable_pin = 27  # GPIO pin for right motor enable

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_enable_pin, GPIO.OUT)
        GPIO.setup(self.right_enable_pin, GPIO.OUT)
        GPIO.output(self.left_enable_pin, GPIO.LOW)
        GPIO.output(self.right_enable_pin, GPIO.LOW)

        # --- PCA9685 Setup for PWM Control ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60  # Common for motor control

        self.left_pwm_channel = 0
        self.right_pwm_channel = 1

        self.set_motor_speed(0, 0)  # Stop motors initially

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.test_state = 0

        self.get_logger().info("Motor test node initialized.")

    def convert_speed_to_duty(self, speed_8bit):
        """
        Convert 8-bit speed value (0–255) to 16-bit duty cycle (0–65535)
        """
        speed_8bit = max(0, min(255, speed_8bit))  # Clamp to 0–255
        return int((speed_8bit / 255.0) * 65535)

    def set_motor_speed(self, left_speed, right_speed):
        """
        Set motor PWM using 8-bit values and convert to 16-bit for PCA9685
        """
        left_duty = self.convert_speed_to_duty(left_speed)
        right_duty = self.convert_speed_to_duty(right_speed)
        self.pca.channels[self.left_pwm_channel].duty_cycle = left_duty
        self.pca.channels[self.right_pwm_channel].duty_cycle = right_duty

    def timer_callback(self):
        if self.test_state == 0:
            self.get_logger().info("State 0: Enabling motors at medium speed.")
            GPIO.output(self.left_enable_pin, GPIO.HIGH)
            GPIO.output(self.right_enable_pin, GPIO.HIGH)
            self.set_motor_speed(120, 120)  # Medium speed
            self.test_state = 1

        elif self.test_state == 1:
            self.get_logger().info("State 1: Increasing motor speed.")
            self.set_motor_speed(220, 220)  # High speed
            self.test_state = 2

        elif self.test_state == 2:
            self.get_logger().info("State 2: Stopping motors.")
            self.set_motor_speed(0, 0)
            GPIO.output(self.left_enable_pin, GPIO.LOW)
            GPIO.output(self.right_enable_pin, GPIO.LOW)
            self.test_state = 0

    def destroy_node(self):
        self.pca.deinit()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
