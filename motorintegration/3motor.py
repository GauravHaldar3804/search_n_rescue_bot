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
        self.enable_pins = {
            'motor1_r': 17,  # R_EN for Motor 1
            'motor1_l': 27,  # L_EN for Motor 1
            'motor2_r': 5,  # R_EN for Motor 2
            'motor2_l': 6,  # L_EN for Motor 2
            'motor3_r': 13,  # R_EN for Motor 3
            'motor3_l': 19   # L_EN for Motor 3
        }

        GPIO.setmode(GPIO.BCM)
        for pin in self.enable_pins.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)  # Enable all motors initially

        # --- PCA9685 Setup for PWM Control ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # Set to 1 kHz for smoother motor control

        # PWM channels for three motors (RPWM and LPWM per motor)
        self.pwm_channels = {
            'motor1_r': 0,  # Right PWM for Motor 1
            'motor1_l': 1,  # Left PWM for Motor 1
            'motor2_r': 2,  # Right PWM for Motor 2
            'motor2_l': 3,  # Left PWM for Motor 2
            'motor3_r': 4,  # Right PWM for Motor 3
            'motor3_l': 5   # Left PWM for Motor 3
        }

        self.set_all_motor_speeds(0, 1)  # Stop all motors initially

        self.start_time = time.time()

        self.get_logger().info("Motor control node initialized for 3 motors with terminal input. Lift motors before starting! Use 'F <speed>', 'B <speed>', or 'S'.")

    def convert_speed_to_duty(self, speed_8bit):
        """Convert 8-bit speed value (0–255) to 16-bit duty cycle (0–65535)"""
        speed_8bit = max(0, min(255, speed_8bit))  # Clamp to 0–255
        return int((speed_8bit / 255.0) * 65535)

    def set_motor_speed(self, motor_id, speed, direction=1):
        """Set PWM for a specific motor with direction control"""
        pwm_r = self.pwm_channels[f'motor{motor_id}_r']
        pwm_l = self.pwm_channels[f'motor{motor_id}_l']
        duty = self.convert_speed_to_duty(speed)
        if direction == 1:  # Forward
            self.pca.channels[pwm_r].duty_cycle = duty
            self.pca.channels[pwm_l].duty_cycle = 0
            print(f"Motor {motor_id} Forward: RPWM={duty/65535*100:.1f}%, LPWM=0%")
        else:  # Reverse
            self.pca.channels[pwm_r].duty_cycle = 0
            self.pca.channels[pwm_l].duty_cycle = duty
            print(f"Motor {motor_id} Reverse: RPWM=0%, LPWM={duty/65535*100:.1f}%")

    def set_all_motor_speeds(self, speed, direction=1):
        """Set PWM for all three motors with the same speed and direction"""
        for motor_id in [1, 2, 3]:
            self.set_motor_speed(motor_id, speed, direction)

    def soft_start(self, target_speed, direction=1):
        """Gradually ramp up speed for all motors to avoid jerk"""
        for speed in range(0, target_speed + 1, 10):
            self.set_all_motor_speeds(speed, direction)
            time.sleep(0.05)  # 500ms ramp-up

    def run(self):
        while rclpy.ok():
            if time.time() - self.start_time >= 30 * 60:  # 30-minute limit
                self.get_logger().info("30-Minute Runtime Limit Reached. Stopping.")
                self.set_all_motor_speeds(0, 1)
                for pin in self.enable_pins.values():
                    GPIO.output(pin, GPIO.LOW)
                break
            try:
                command = input().strip().split()
                if len(command) == 2 and command[0] in ['F', 'B'] and command[1].isdigit():
                    speed = int(command[1])
                    direction = 1 if command[0] == 'F' else -1
                    self.get_logger().info(f"Received: All motors {'Forward' if direction == 1 else 'Reverse'} at speed {speed}")
                    self.soft_start(speed, direction)
                elif command[0] == 'S':
                    self.get_logger().info("Received: Stop")
                    self.set_all_motor_speeds(0, 1)
                else:
                    self.get_logger().info("Invalid command! Use 'F <speed>', 'B <speed>', or 'S'")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.set_all_motor_speeds(0, 1)
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