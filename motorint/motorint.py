import RPi.GPIO as GPIO
import time

# Pin definitions
IN1 = 17  # Forward PWM (RPWM)
IN2 = 27  # Reverse PWM (LPWM)
EN = 22   # Enable pin

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(EN, GPIO.OUT)

# Initialize PWM on IN1 and IN2 with 1 kHz frequency
pwm_forward = GPIO.PWM(IN1, 1000)  # 1000 Hz
pwm_reverse = GPIO.PWM(IN2, 1000)  # 1000 Hz

# Start PWM with 0% duty cycle (motor off)
pwm_forward.start(0)
pwm_reverse.start(0)

# Enable the motor driver
GPIO.output(EN, GPIO.HIGH)

def set_motor_speed(direction, speed):
    """
    Control motor direction and speed
    :param direction: 'forward' or 'reverse'
    :param speed: 0-100 (percentage of max speed)
    """
    if direction == "forward":
        pwm_forward.ChangeDutyCycle(speed)
        pwm_reverse.ChangeDutyCycle(0)
    elif direction == "reverse":
        pwm_forward.ChangeDutyCycle(0)
        pwm_reverse.ChangeDutyCycle(speed)
    else:
        print("Invalid direction! Use 'forward' or 'reverse'")

def stop_motor():
    """Stop the motor"""
    pwm_forward.ChangeDutyCycle(0)
    pwm_reverse.ChangeDutyCycle(0)

# Example usage
try:
    while True:
        # Forward at 50% speed for 2 seconds
        print("Moving forward at 50% speed")
        set_motor_speed("forward", 50)
        time.sleep(2)

        # Stop for 1 second
        print("Stopping")
        stop_motor()
        time.sleep(1)

        # Reverse at 75% speed for 2 seconds
        print("Moving reverse at 75% speed")
        set_motor_speed("reverse", 75)
        time.sleep(2)

        # Stop for 1 second
        print("Stopping")
        stop_motor()
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up on Ctrl+C
    stop_motor()
    pwm_forward.stop()
    pwm_reverse.stop()
    GPIO.cleanup()
    print("Program terminated")