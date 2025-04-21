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
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # GPIO Setup for Enable Pins
        self.enable_pins = {
            'group1_3_r': 17,  # Right enables for motors 1, 2, 3
            'group1_3_l': 27,  # Left enables for motors 1, 2, 3
            'group4_6_r': 20,  # Right enables for motors 4, 5, 6
            'group4_6_l': 16   # Left enables for motors 4, 5, 6
        }
        GPIO.setmode(GPIO.BCM)
        for pin in self.enable_pins.values():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)

        # PCA9685 Setup
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000

        # PWM channels (corrected as per your input)
        self.pwm_channels = {
            'motor1_r': 0, 'motor1_l': 1,
            'motor2_r': 2, 'motor2_l': 3,
            'motor3_r': 4, 'motor3_l': 5,
            'motor4_r': 6, 'motor4_l': 7,
            'motor5_r': 10, 'motor5_l': 11,
            'motor6_r': 8, 'motor6_l': 9
        }

        # Encoder counts and PID state
        self.encoder_counts = [0] * 6
        self.pulses_per_rev = 2130
        self.pulses_per_180 = 1065  # Half of 2130 for 180 degrees
        self.integral = [0] * 6
        self.prev_error = [0] * 6
        self.pid_outputs = [0] * 6
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.01  # Integral gain
        self.Kd = 0.1  # Derivative gain
        self.max_speed = 255  # Maximum speed (8-bit)

        self.set_all_motor_speeds(0, 1)
        self.get_logger().info("Motor control initialized. Use 'A' to move to 180 degrees with PID, 'S' to stop.")

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

    def compute_pid(self, motor_id):
        idx = motor_id - 1
        error = self.pulses_per_180 - abs(self.encoder_counts[idx])
        self.integral[idx] += error * 0.01  # Integral term with time step
        derivative = (error - self.prev_error[idx]) / 0.01  # Derivative term
        self.pid_outputs[idx] = (self.Kp * error) + (self.Ki * self.integral[idx]) + (self.Kd * derivative)
        self.prev_error[idx] = error
        speed = min(self.max_speed, max(0, int(self.pid_outputs[idx])))
        return speed

    def soft_start(self, target_speed, direction=1, motor_ids=None):
        if motor_ids is None:
            motor_ids = range(1, 7)
        for speed in range(0, target_speed + 1, 10):
            for motor_id in motor_ids:
                self.set_motor_speed(motor_id, speed, direction)
            time.sleep(0.05)

    def read_encoders(self):
        self.serial_port.write("READ\n".encode())
        response = self.serial_port.readline().decode().strip()
        if response:
            try:
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
            self.integral = [0] * 6
            self.prev_error = [0] * 6
            self.pid_outputs = [0] * 6
            self.get_logger().info("Encoders and PID state reset to zero")
            return True
        return False

    def move_to_180_degrees(self, speed=50):
        self.get_logger().info("Moving motors 2, 3, 5, 6 to 180 degrees with PID")
        self.reset_encoders()
        active_motors = [2, 3, 5, 6]
        GPIO.output(self.enable_pins['group1_3_r'], GPIO.HIGH)
        GPIO.output(self.enable_pins['group1_3_l'], GPIO.HIGH)
        GPIO.output(self.enable_pins['group4_6_r'], GPIO.HIGH)
        GPIO.output(self.enable_pins['group4_6_l'], GPIO.HIGH)
        self.soft_start(speed, direction=1, motor_ids=active_motors)

        start_time = time.time()
        max_duration = 30  # Maximum runtime in seconds to prevent infinite loops

        while time.time() - start_time < max_duration:
            if not self.read_encoders():
                self.get_logger().error("Failed to read encoders")
                break

            all_at_target = True
            for i in active_motors:
                idx = i - 1
                if abs(self.encoder_counts[idx]) < self.pulses_per_180:
                    all_at_target = False
                    speed = self.compute_pid(i)
                    self.set_motor_speed(i, speed, 1)
                else:
                    self.set_motor_speed(i, 0, 1)
                    self.get_logger().info(f"Motor {i} reached 180 degrees at {self.encoder_counts[idx]} pulses")

            if all_at_target:
                break
            time.sleep(0.01)

        self.set_all_motor_speeds(0, 1)
        GPIO.output(self.enable_pins['group1_3_r'], GPIO.LOW)
        GPIO.output(self.enable_pins['group1_3_l'], GPIO.LOW)
        GPIO.output(self.enable_pins['group4_6_r'], GPIO.LOW)
        GPIO.output(self.enable_pins['group4_6_l'], GPIO.LOW)
        self.get_logger().info("Motors 2, 3, 5, 6 at 180 degrees")

    def run(self):
        while rclpy.ok():
            try:
                command = input().strip().split()
                if command[0] == 'A':
                    self.move_to_180_degrees()
                elif command[0] == 'S':
                    self.get_logger().info("Stopping all motors")
                    self.set_all_motor_speeds(0, 1)
                    GPIO.output(self.enable_pins['group1_3_r'], GPIO.LOW)
                    GPIO.output(self.enable_pins['group1_3_l'], GPIO.LOW)
                    GPIO.output(self.enable_pins['group4_6_r'], GPIO.LOW)
                    GPIO.output(self.enable_pins['group4_6_l'], GPIO.LOW)
                else:
                    self.get_logger().info("Invalid command! Use 'A' to move to 180 degrees or 'S' to stop")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.set_all_motor_speeds(0, 1)
        GPIO.output(self.enable_pins['group1_3_r'], GPIO.LOW)
        GPIO.output(self.enable_pins['group1_3_l'], GPIO.LOW)
        GPIO.output(self.enable_pins['group4_6_r'], GPIO.LOW)
        GPIO.output(self.enable_pins['group4_6_l'], GPIO.LOW)
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
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()