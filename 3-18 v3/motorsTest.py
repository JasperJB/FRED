#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

MOTOR_PINS = {
    "ENA": 12,
    "ENB": 13,
    "IN1": 17,
    "IN2": 18,
    "IN3": 27,
    "IN4": 22,
}

def main():
    # Configure GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)

    # Enable motor power
    GPIO.output(MOTOR_PINS["ENA"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["ENB"], GPIO.HIGH)

    # Drive motors forward (IN1/IN3 high, IN2/IN4 low)
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)

    # Run for 3 seconds
    time.sleep(3)

    # Stop motors
    GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)

    # Clean up
    GPIO.cleanup()

if __name__ == "__main__":
    main()
