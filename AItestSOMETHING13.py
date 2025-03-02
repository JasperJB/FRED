#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import time
import RPi.GPIO as GPIO
import pickle
import random
import math
import os

#############################################
# Motor & GPIO Setup
#############################################
MOTOR_PINS = {
    "ENA": 12,  # Right motor PWM
    "ENB": 13,  # Left motor PWM
    "IN1": 17,  # Right motor direction
    "IN2": 18,
    "IN3": 27,  # Left motor direction
    "IN4": 22,
}

RESET_PIN = 14  # GPIO14 used to reset the Arduino

GPIO.setmode(GPIO.BCM)
# Setup motor control pins
for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
# Setup reset pin
GPIO.setup(RESET_PIN, GPIO.OUT)
GPIO.output(RESET_PIN, GPIO.LOW)

# Set up PWM on the enable pins (frequency in Hz)
pwm_right = GPIO.PWM(MOTOR_PINS["ENA"], 1000)
pwm_left = GPIO.PWM(MOTOR_PINS["ENB"], 1000)
pwm_right.start(0)
pwm_left.start(0)

def set_motor_speed(left_speed, right_speed, left_direction='forward', right_direction='forward'):
    """
    Sets the motor speeds and directions.
    Speeds: 0 to 100 (percentage)
    Directions: 'forward' or 'backward'
    """
    # Left motor direction (IN3 & IN4)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH if left_direction == 'forward' else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW if left_direction == 'forward' else GPIO.HIGH)
    # Right motor direction (IN1 & IN2)
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH if right_direction == 'forward' else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW if right_direction == 'forward' else GPIO.HIGH)
    # Set PWM speeds
    pwm_left.ChangeDutyCycle(left_speed)
    pwm_right.ChangeDutyCycle(right_speed)

def stop_motors():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

#############################################
# Auto-detect Arduino Serial Port
#############################################
def find_arduino():
    """
    Scans available serial ports to locate the Arduino.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    raise RuntimeError("Arduino not found.")

#############################################
# PID Controller for Heading
#############################################
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def update(self, current_value, dt):
        error = current_value - self.setpoint
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

#############################################
# Main Loop
#############################################
def main():
    # Trigger Arduino reset via GPIO14
    print("Resetting Arduino via GPIO14...")
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(2)  # Allow time for Arduino to reset and re-enumerate

    # --- Set up Serial ---
    try:
        arduino_port = find_arduino()
    except RuntimeError as e:
        print(str(e))
        return

    try:
        ser = serial.Serial(arduino_port, 9600, timeout=1)
    except Exception as e:
        print("Error opening serial port:", e)
        return

    # Wait for Arduino calibration message
    print("Waiting for Arduino calibration...")
    calibrated = False
    while not calibrated:
        try:
            line = ser.readline().decode().strip()
        except serial.SerialException as e:
            print("Serial exception:", e)
            time.sleep(0.5)
            continue
        if "Calibrated" in line:
            calibrated = True
            print("Arduino calibrated. Starting main loop.")

    # --- Initialize PID Controller ---
    pid = PIDController(Kp=40.0, Ki=0.0, Kd=4.0, setpoint=0)
    prev_time = time.time()

    # --- Main Loop ---
    try:
        while True:
            # Send a "P" ping for data
            ser.write(b'P')
            line = ser.readline().decode().strip()
            if not line:
                continue

            # Parse data from Arduino
            parts = line.split(',')
            if len(parts) < 6:  # Expecting 5 sensors + heading
                continue
            try:
                distances = [float(parts[i]) for i in range(5)]  # Read 5 sensors
                current_heading = float(parts[5])  # Read heading
            except Exception as e:
                print("Error parsing sensor data:", e)
                continue

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # --- Obstacle Evaluation ---
            front_sensors = distances[:3]
            if any(d < 35 for d in front_sensors):
                print("Obstacle detected! Pivoting.")
                left_clearance = front_sensors[0]
                right_clearance = front_sensors[2]

                # Pivot towards the side with more space
                chosen_action = 1 if right_clearance > left_clearance else 0

                # Execute action
                if chosen_action == 0:
                    set_motor_speed(100, 100, left_direction='backward', right_direction='forward')  # Pivot left
                else:
                    set_motor_speed(100, 100, left_direction='forward', right_direction='backward')  # Pivot right
                
                time.sleep(0.8)
                stop_motors()
                print("Pivot complete. Resuming forward motion.")
                # Reset PID controller to prefer returning to a forward heading
                pid.integral = 0
                pid.prev_error = 0
                pid.setpoint = 0

            # --- PID Heading Control ---
            else:
                pid_output = pid.update(current_heading, dt)
                if pid_output > 0:
                    left_speed = 100
                    right_speed = 100 - pid_output
                else:
                    left_speed = 100 + pid_output
                    right_speed = 100

                left_speed = max(0, min(100, left_speed))
                right_speed = max(0, min(100, right_speed))

                set_motor_speed(left_speed, right_speed)
                print(f"Heading: {current_heading:.2f}, PID: {pid_output:.2f}, Left: {left_speed:.2f}%, Right: {right_speed:.2f}%")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Stopping the robot.")
    finally:
        stop_motors()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
