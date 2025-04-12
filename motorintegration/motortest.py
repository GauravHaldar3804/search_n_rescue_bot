#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # --- Pin Definitions ---
        self.RPWM = 18  # PWM output for forward direction
        self.LPWM = 23  # PWM output for reverse direction
        self.ENL = 17   # Enable left
        self.ENR = 27   # Enable right

        # --- GPIO Setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RPWM, GPIO.OUT)
        GPIO.setup(self.LPWM, GPIO.OUT)
        GPIO.setup(self.ENL, GPIO.OUT)
        GPIO.setup(self.ENR, GPIO.OUT)

        # Enable motor driver
        GPIO.output(self.ENL, GPIO.HIGH)
        GPIO.output(self.ENR, GPIO.HIGH)

        # Setup PWM at 1000Hz
        self.pwm_r = GPIO.PWM(self.RPWM, 1000)
        self.pwm_l = GPIO.PWM(self.LPWM, 1000)

        self.pwm_r.start(0)
        self.pwm_l.start(0)

        # ROS 2 Subscription
        self.subscription = self.create_subscription(
            Int32,
            'motor_speed',
            self.speed_callback,
            10
        )

        self.get_logger().info("Motor control node initialized.")

    def forward(self, speed_percent):
        """Move motor forward with specified speed"""
        self.get_logger().info(f"Moving forward at {speed_percent}% speed")
        self.pwm_l.ChangeDutyCycle(0)
        self.pwm_r.ChangeDutyCycle(speed_percent)

    def backward(self, speed_percent):
        """Move motor backward with specified speed"""
        self.get_logger().info(f"Moving backward at {speed_percent}% speed")
        self.pwm_r.ChangeDutyCycle(0)
        self.pwm_l.ChangeDutyCycle(speed_percent)

    def stop(self):
        """Stop the motor"""
        self.get_logger().info("Stopping motor")
        self.pwm_r.ChangeDutyCycle(0)
        self.pwm_l.ChangeDutyCycle(0)

    def speed_callback(self, msg):
        """Callback function that controls motor speed"""
        speed = msg.data  # Get the speed value from message
        if speed > 0:
            self.forward(speed)
        elif speed < 0:
            self.backward(abs(speed))
        else:
            self.stop()

    def destroy_node(self):
        """Cleanup GPIO and stop PWM"""
        self.stop()
        self.pwm_r.stop()
        self.pwm_l.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
