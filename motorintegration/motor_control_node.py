#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import serial
import RPi.GPIO as GPIO
import board
import busio
from adafruit_pca9685 import PCA9685

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Serial communication with Arduino
        port = '/dev/ttyUSB0'
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # GPIO Setup for BTS7960 Enable Pins
        self.enable_pins = {
            'motor1_r': 17, 'motor1_l': 27,
            'motor2_r': 5, 'motor2_l': 6,
            'motor3_r': 13, 'motor3_l': 19,
            'motor4_r': 20, 'motor4_l': 16,
            'motor5_r': 20, 'motor5_l': 16,
            'motor6_r': 20, 'motor6_l': 16
        }
        GPIO.setmode(GPIO.BCM)
        for pin in set(self.enable_pins.values()):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)

        # PCA9685 Setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000

        # PWM channels
        self.pwm_channels = {
            'motor1_r': 0, 'motor1_l': 1,
            'motor2_r': 2, 'motor2_l': 3,
            'motor3_r': 4, 'motor3_l': 5,
            'motor4_r': 6, 'motor4_l': 7,
            'motor5_r': 8, 'motor5_l': 9,
            'motor6_r': 10, 'motor6_l': 11
        }

        # Encoder counts (from Arduino)
        self.encoder_counts = [0] * 6
        self.pulses_per_90 = 533  # 2130 PPR / 4

        self.set_all_motor_speeds(0, 1)  # Stop all motors initially
        self.get_logger().info("Motor control node initialized. Lift motors before starting! Use 'A' to move to 90 degrees, 'S' to stop.")

    def convert_speed_to_duty(self, speed_8bit):
        speed_8bit = max(0, min(255, speed_8bit))
        return int((speed_8bit / 255.0) * 65535)

    def set_motor_speed(self, motor_id, speed, direction=1):
        pwm_r = self.pwm_channels[f'motor{motor_id}_r']
        pwm_l = self.pwm_channels[f'motor{motor_id}_l']
        duty = self.convert_speed_to_duty(speed)
        if direction == 1:
            self.pca.channels[pwm_r].duty_cycle = duty
            self.pca.channels[pwm_l].duty_cycle = 0
        else:
            self.pca.channels[pwm_r].duty_cycle = 0
            self.pca.channels[pwm_l].duty_cycle = duty

    def set_all_motor_speeds(self, speed, direction=1):
        for motor_id in range(1, 7):
            self.set_motor_speed(motor_id, speed, direction)

    def soft_start(self, target_speed, direction=1, motor_ids=None):
        if motor_ids is None:
            motor_ids = range(1, 7)
        for speed in range(0, target_speed + 1, 10):
            for motor_id in motor_ids:
                if motor_id not in [1, 4]:  # Ignore motors 1 and 4
                    self.set_motor_speed(motor_id, speed, direction)
            time.sleep(0.05)

    def read_encoders(self):
        self.serial_port.write("READ\n".encode())
        response = self.serial_port.readline().decode().strip()
        if response:
            try:
                # Parse Enc1:123,Enc2:456,...
                counts = response.split(',')
                for i, count in enumerate(counts):
                    if count.startswith(f"Enc{i+1}:"):
                        self.encoder_counts[i] = int(count.split(':')[1])
                self.get_logger().info(f"Encoder counts: {self.encoder_counts}")
                return True
            except Exception as e:
                self.get_logger().error(f"Error parsing encoder data: {e}")
        return False

    def reset_encoders(self):
        self.serial_port.write("RESET\n".encode())
        response = self.serial_port.readline().decode().strip()
        if response == "RESET_OK":
            self.encoder_counts = [0] * 6
            self.get_logger().info("Encoders reset to home position")
            return True
        return False

    def move_to_90_degrees(self, speed=50):
        self.get_logger().info("Moving motors 2, 3, 5, 6 to 90 degrees")
        self.reset_encoders()  # Ensure all start from zero
        active_motors = [2, 3, 5, 6]  # Ignore motors 1 and 4
        self.soft_start(speed, direction=1, motor_ids=active_motors)
        target_pulses = self.pulses_per_90
        while any(abs(self.encoder_counts[i-1]) < target_pulses for i in active_motors):
            if not self.read_encoders():
                self.get_logger().error("Failed to read encoders")
                break
            for i in active_motors:
                if abs(self.encoder_counts[i-1]) >= target_pulses:
                    self.set_motor_speed(i, 0, 1)  # Stop motor
            time.sleep(0.01)
        self.set_all_motor_speeds(0, 1)  # Ensure all stop
        self.get_logger().info("Motors 2, 3, 5, 6 at 90 degrees")

    def run(self):
        while rclpy.ok():
            try:
                command = input().strip().split()
                if command[0] == 'A':
                    self.get_logger().info("Starting move to 90 degrees")
                    self.move_to_90_degrees()
                elif command[0] == 'S':
                    self.get_logger().info("Stop")
                    self.set_all_motor_speeds(0, 1)
                else:
                    self.get_logger().info("Invalid command! Use 'A' to move to 90 degrees or 'S' to stop")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.set_all_motor_speeds(0, 1)
        self.pca.deinit()
        GPIO.cleanup()
        self.serial_port.close()
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