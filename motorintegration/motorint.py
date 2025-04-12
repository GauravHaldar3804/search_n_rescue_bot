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
        self.enable_r_pin = 17  # Right enable (R_EN)
        self.enable_l_pin = 27  # Left enable (L_EN)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enable_r_pin, GPIO.OUT)
        GPIO.setup(self.enable_l_pin, GPIO.OUT)
        GPIO.output(self.enable_r_pin, GPIO.HIGH)  # Enable motor initially
        GPIO.output(self.enable_l_pin, GPIO.HIGH)

        # --- PCA9685 Setup for PWM Control ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # Set to 1 kHz for smoother motor control

        # PWM channels for one motor (RPWM and LPWM)
        self.pwm_r = 0  # Right PWM (forward)
        self.pwm_l = 1  # Left PWM (reverse)

        self.set_motor_speed(0, 1)  # Stop motor initially

        self.timer = self.create_timer(0.1, self.timer_callback)  # Faster callback
        self.test_state = 0
        self.start_time = time.time()

        self.get_logger().info("Motor test node initialized for one motor with PCA9685. Lift motor before starting!")

    def convert_speed_to_duty(self, speed_8bit):
        """Convert 8-bit speed value (0–255) to 16-bit duty cycle (0–65535)"""
        speed_8bit = max(0, min(255, speed_8bit))  # Clamp to 0–255
        return int((speed_8bit / 255.0) * 65535)

    def set_motor_speed(self, speed, direction=1):
        """Set motor PWM with direction control (1 for forward, -1 for reverse)"""
        duty = self.convert_speed_to_duty(speed)
        if direction == 1:  # Forward
            self.pca.channels[self.pwm_r].duty_cycle = duty
            self.pca.channels[self.pwm_l].duty_cycle = 0
            print(f"Forward: RPWM={duty/65535*100:.1f}%, LPWM=0%")
        else:  # Reverse
            self.pca.channels[self.pwm_r].duty_cycle = 0
            self.pca.channels[self.pwm_l].duty_cycle = duty
            print(f"Reverse: RPWM=0%, LPWM={duty/65535*100:.1f}%")

    def soft_start(self, target_speed, direction=1):
        """Gradually ramp up motor speed to avoid jerk"""
        for speed in range(0, target_speed + 1, 10):
            self.set_motor_speed(speed, direction)
            time.sleep(0.05)  # 500ms ramp-up

    def timer_callback(self):
        current_time = time.time()
        if current_time - self.start_time >= 30 * 60:  # 30-minute limit
            self.get_logger().info("30-Minute Runtime Limit Reached. Stopping.")
            self.set_motor_speed(0, 1)
            GPIO.output(self.enable_r_pin, GPIO.LOW)
            GPIO.output(self.enable_l_pin, GPIO.LOW)
            return

        if self.test_state == 0:
            self.get_logger().info("State 0: Starting motor with soft start.")
            self.soft_start(120, 1)  # Medium speed forward
            self.test_state = 1

        elif self.test_state == 1:
            self.get_logger().info("State 1: Increasing speed.")
            self.soft_start(220, 1)  # High speed forward
            self.test_state = 2

        elif self.test_state == 2:
            self.get_logger().info("State 2: Reversing motor.")
            self.soft_start(120, -1)  # Medium speed reverse
            self.test_state = 3

        elif self.test_state == 3:
            self.get_logger().info("State 3: Stopping motor.")
            self.set_motor_speed(0, 1)
            self.test_state = 0
            self.start_time = time.time()  # Reset timer for next cycle

    def destroy_node(self):
        self.set_motor_speed(0, 1)
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