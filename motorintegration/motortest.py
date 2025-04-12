#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys

# GPIO setup (BCM mode)
GPIO.setmode(GPIO.BCM)

# Define pins for one BTS7960 motor driver
MOTOR = {
    "RPWM": 12,  # Right PWM (forward)
    "LPWM": 13,  # Left PWM (reverse)
    "R_EN": 17,  # Right enable
    "L_EN": 27   # Left enable
}

# Initialize GPIO pins
GPIO.setup(MOTOR["RPWM"], GPIO.OUT)
GPIO.setup(MOTOR["LPWM"], GPIO.OUT)
GPIO.setup(MOTOR["R_EN"], GPIO.OUT)
GPIO.setup(MOTOR["L_EN"], GPIO.OUT)

# Set up PWM with 1 kHz frequency
pwm_r = GPIO.PWM(MOTOR["RPWM"], 1000)
pwm_l = GPIO.PWM(MOTOR["LPWM"], 1000)
pwm_r.start(0)
pwm_l.start(0)

# Enable motor (both directions initially)
GPIO.output(MOTOR["R_EN"], GPIO.HIGH)
GPIO.output(MOTOR["L_EN"], GPIO.HIGH)

MAX_RUNTIME = 30 * 60  # 30 minutes in seconds
start_time = time.time()

def convert_speed_to_duty(speed_8bit):
    """Convert 8-bit speed (0-255) to duty cycle (0-100)"""
    speed_8bit = max(0, min(255, speed_8bit))
    return int((speed_8bit / 255.0) * 100)

def soft_start(target_speed, direction=1):
    """Gradually ramp up motor speed to avoid jerk"""
    for speed in range(0, target_speed + 1, 10):
        duty = convert_speed_to_duty(speed)
        if direction == 1:  # Forward
            pwm_r.ChangeDutyCycle(duty)
            pwm_l.ChangeDutyCycle(0)
            print(f"Forward: RPWM={duty}%, LPWM=0%")
        else:  # Reverse
            pwm_r.ChangeDutyCycle(0)
            pwm_l.ChangeDutyCycle(duty)
            print(f"Reverse: RPWM=0%, LPWM={duty}%")
        time.sleep(0.05)  # 500ms ramp-up

def set_motor_speed(speed, direction=1):
    """Set motor speed and direction (1 for forward, -1 for reverse)"""
    duty = convert_speed_to_duty(speed)
    if direction == 1:  # Forward
        pwm_r.ChangeDutyCycle(duty)
        pwm_l.ChangeDutyCycle(0)
        print(f"Forward: RPWM={duty}%, LPWM=0%")
    else:  # Reverse
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(duty)
        print(f"Reverse: RPWM=0%, LPWM={duty}%")

def stop_motor():
    """Stop the motor"""
    pwm_r.ChangeDutyCycle(0)
    pwm_l.ChangeDutyCycle(0)
    print("Motor stopped: RPWM=0%, LPWM=0%")

try:
    print("Single Motor Control Started")
    print("Enter 'F <speed>' for Forward, 'B <speed>' for Backward, 'S' to Stop")
    print("Lift motor before starting! Speed range: 0-255")
    print("Example: F 120")

    while True:
        if time.time() - start_time >= MAX_RUNTIME:
            print("30-Minute Runtime Limit Reached. Stopping.")
            stop_motor()
            break

        command = input().strip().split()
        if len(command) == 2 and command[0] in ['F', 'B'] and command[1].isdigit():
            speed = int(command[1])
            direction = 1 if command[0] == 'F' else -1
            print(f"Setting motor {'Forward' if direction == 1 else 'Reverse'} at speed {speed}")
            soft_start(speed, direction)
        elif command[0] == 'S':
            print("Stopping motor")
            stop_motor()
        else:
            print("Invalid command! Use 'F <speed>', 'B <speed>', or 'S'")

except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    stop_motor()
    pwm_r.stop()
    pwm_l.stop()
    GPIO.cleanup()