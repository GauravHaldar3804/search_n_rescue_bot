#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

# Import libraries to control GPIO and PCA9685 for PWM
import RPi.GPIO as GPIO
import board
import busio
from adafruit_pca9685 import PCA9685

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

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

        self.start_time = time.time()

        self.get_logger().info("Motor control node initialized with terminal input. Lift motor before starting! Use 'F <speed>', 'B <speed>', or 'S'.")

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

    def run(self):
        while rclpy.ok():
            if time.time() - self.start_time >= 30 * 60:  # 30-minute limit
                self.get_logger().info("30-Minute Runtime Limit Reached. Stopping.")
                self.set_motor_speed(0, 1)
                GPIO.output(self.enable_r_pin, GPIO.LOW)
                GPIO.output(self.enable_l_pin, GPIO.LOW)
                break
            try:
                command = input().strip().split()
                if len(command) == 2 and command[0] in ['F', 'B'] and command[1].isdigit():
                    speed = int(command[1])
                    direction = 1 if command[0] == 'F' else -1
                    self.get_logger().info(f"Received: {'Forward' if direction == 1 else 'Reverse'} at speed {speed}")
                    self.soft_start(speed, direction)
                elif command[0] == 'S':
                    self.get_logger().info("Received: Stop")
                    self.set_motor_speed(0, 1)
                else:
                    self.get_logger().info("Invalid command! Use 'F <speed>', 'B <speed>', or 'S'")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.set_motor_speed(0, 1)
        self.pca.deinit()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()