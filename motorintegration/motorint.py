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
        # Define the GPIO pins (BCM numbering) for the left and right enable.
        # Adjust these numbers based on your actual wiring.
        self.left_enable_pin = 17   # Example GPIO pin for left motor enable
        self.right_enable_pin = 27  # Example GPIO pin for right motor enable

        # Set up the GPIO mode and pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_enable_pin, GPIO.OUT)
        GPIO.setup(self.right_enable_pin, GPIO.OUT)
        GPIO.output(self.left_enable_pin, GPIO.LOW)
        GPIO.output(self.right_enable_pin, GPIO.LOW)

        # --- PCA9685 Setup for PWM Control ---
        # Initialize I2C bus using default SDA and SCL
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 60  # Set PWM frequency (60Hz is common for motor control)

        # Define which PCA9685 channels to use for each motor's PWM control.
        self.left_pwm_channel = 0   # Example: Channel 0 for left motor PWM
        self.right_pwm_channel = 1  # Example: Channel 1 for right motor PWM

        # Set initial PWM values to zero.
        self.pca.channels[self.left_pwm_channel].duty_cycle = 0
        self.pca.channels[self.right_pwm_channel].duty_cycle = 0

        # --- Timer to Run the Test ---
        # Create a timer callback that changes motor states every few seconds.
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.test_state = 0

        self.get_logger().info("Motor test node initialized.")

    def timer_callback(self):
        """
        Simple state machine for testing:
          State 0: Enable both motors with a medium PWM value.
          State 1: Increase PWM value to test a higher speed.
          State 2: Stop motors (disable and PWM = 0).
        """
        if self.test_state == 0:
            self.get_logger().info("State 0: Enabling motors at medium speed.")
            # Enable both motor drivers
            GPIO.output(self.left_enable_pin, GPIO.HIGH)
            GPIO.output(self.right_enable_pin, GPIO.HIGH)
            # Set a medium PWM value (range 0-65535; PCA9685 uses 16-bit values)
            # Note: duty_cycle values can also be set as 0-4095 if using a 12-bit abstraction,
            # but here we assume full 16-bit values. Adjust accordingly.
            medium_speed = 30000  
            self.pca.channels[self.left_pwm_channel].duty_cycle = medium_speed
            self.pca.channels[self.right_pwm_channel].duty_cycle = medium_speed
            self.test_state = 1

        elif self.test_state == 1:
            self.get_logger().info("State 1: Increasing motor speed.")
            # Increase PWM value for higher speed.
            high_speed = 50000
            self.pca.channels[self.left_pwm_channel].duty_cycle = high_speed
            self.pca.channels[self.right_pwm_channel].duty_cycle = high_speed
            self.test_state = 2

        elif self.test_state == 2:
            self.get_logger().info("State 2: Stopping motors.")
            # Stop the motors: disable driver outputs and set PWM to 0.
            self.pca.channels[self.left_pwm_channel].duty_cycle = 0
            self.pca.channels[self.right_pwm_channel].duty_cycle = 0
            GPIO.output(self.left_enable_pin, GPIO.LOW)
            GPIO.output(self.right_enable_pin, GPIO.LOW)
            self.test_state = 0

    def destroy_node(self):
        # Clean up the PCA9685 and GPIO when the node is destroyed.
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
