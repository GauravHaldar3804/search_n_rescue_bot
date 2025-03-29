import RPi.GPIO as GPIO
import time

# Pin definitions for BTS7960 with separate R_EN and L_EN
IN1 = 17  # RPWM: Forward PWM input (Physical Pin 11)
IN2 = 27  # LPWM: Reverse PWM input (Physical Pin 13)
R_EN = 22 # Right Enable (forward) (Physical Pin 15)
L_EN = 23 # Left Enable (reverse) (Physical Pin 16)

# Setup GPIO
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(R_EN, GPIO.OUT)
GPIO.setup(L_EN, GPIO.OUT)

# Initialize PWM on IN1 and IN2 with 1 kHz frequency
pwm_forward = GPIO.PWM(IN1, 1000)  # 1000 Hz
pwm_reverse = GPIO.PWM(IN2, 1000)  # 1000 Hz

# Start PWM with 0% duty cycle (motor off)
pwm_forward.start(0)
pwm_reverse.start(0)

# Enable both half-bridges
GPIO.output(R_EN, GPIO.HIGH)
GPIO.output(L_EN, GPIO.HIGH)

def set_motor_speed(direction, speed):
    """
    Control motor direction and speed
    :param direction: 'forward' or 'reverse'
    :param speed: 0-100 (percentage of max speed)
    """
    speed = max(0, min(100, speed))  # Constrain speed to 0-100
    if direction == "forward":
        pwm_forward.ChangeDutyCycle(speed)
        pwm_reverse.ChangeDutyCycle(0)
        print(f"Forward: {speed}%")
    elif direction == "reverse":
        pwm_forward.ChangeDutyCycle(0)
        pwm_reverse.ChangeDutyCycle(speed)
        print(f"Reverse: {speed}%")
    else:
        print("Invalid direction!")

def stop_motor():
    """Stop the motor"""
    pwm_forward.ChangeDutyCycle(0)
    pwm_reverse.ChangeDutyCycle(0)
    print("Stopped")

# Main loop
try:
    while True:
        set_motor_speed("forward", 50)  # 50% speed forward
        time.sleep(2)
        stop_motor()
        time.sleep(1)
        set_motor_speed("reverse", 75)  # 75% speed reverse
        time.sleep(2)
        stop_motor()
        time.sleep(1)

except KeyboardInterrupt:
    # Clean up on Ctrl+C
    stop_motor()
    pwm_forward.stop()
    pwm_reverse.stop()
    GPIO.cleanup()
    print("Program terminated")