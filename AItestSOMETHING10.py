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
pwm_left  = GPIO.PWM(MOTOR_PINS["ENB"], 1000)
pwm_right.start(0)
pwm_left.start(0)

def set_motor_speed(left_speed, right_speed, left_direction='forward', right_direction='forward'):
    """
    Sets the motor speeds and directions.
    Speeds: 0 to 100 (percentage)
    Directions: 'forward' or 'backward'
    """
    # Left motor direction (IN3 & IN4)
    if left_direction == 'forward':
        GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    else:
        GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)
    # Right motor direction (IN1 & IN2)
    if right_direction == 'forward':
        GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    else:
        GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
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
# PID Controller for Heading (Working Version)
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
# Q-Learning Setup for Obstacle Avoidance
#############################################
Q_TABLE_FILE = "q_table.pkl"
# Actions:
# 0: Pivot left (robot rotates in-place to the left)
# 1: Pivot right (robot rotates in-place to the right)
ACTIONS = [0, 1]

ALPHA = 0.1   # learning rate
GAMMA = 0.9   # discount factor
EPSILON = 0.1 # exploration rate

# Factor to weight heading improvement reward
HEADING_FACTOR = 1.0

def load_q_table():
    try:
        with open(Q_TABLE_FILE, "rb") as f:
            q_table = pickle.load(f)
        print("Q-table loaded.")
        return q_table
    except:
        print("No Q-table found, initializing new one.")
        return {}

def save_q_table(q_table):
    with open(Q_TABLE_FILE, "wb") as f:
        pickle.dump(q_table, f)

def discretize_sensor(value):
    if value < 10:
        return 0
    elif value < 20:
        return 1
    else:
        return 2

def get_state(front_sensors):
    return (discretize_sensor(front_sensors[0]),
            discretize_sensor(front_sensors[1]),
            discretize_sensor(front_sensors[2]))

def choose_action(state, q_table):
    if random.random() < EPSILON:
        return random.choice(ACTIONS)
    else:
        state_actions = q_table.get(state, [0 for _ in ACTIONS])
        max_q = max(state_actions)
        best_actions = [a for a, q in enumerate(state_actions) if q == max_q]
        return random.choice(best_actions)

def update_q_value(q_table, state, action, reward, next_state):
    state_actions = q_table.get(state, [0 for _ in ACTIONS])
    next_state_actions = q_table.get(next_state, [0 for _ in ACTIONS])
    best_next_q = max(next_state_actions)
    new_q = state_actions[action] + ALPHA * (reward + GAMMA * best_next_q - state_actions[action])
    state_actions[action] = new_q
    q_table[state] = state_actions

# --- New execute_action: In-place pivot turns ---
def execute_action(action):
    if action == 0:  
        # Pivot left: left motor goes backward, right motor goes forward
        set_motor_speed(100, 100, left_direction='backward', right_direction='forward')
    elif action == 1:  
        # Pivot right: left motor goes forward, right motor goes backward
        set_motor_speed(100, 100, left_direction='forward', right_direction='backward')

def q_learning_step(action, duration=1.0):
    execute_action(action)
    time.sleep(duration)

#############################################
# Main Loop
#############################################
def main():
    # Variables to track repeated actions
    last_chosen_action = None
    repeat_count = 0
    REPEAT_THRESHOLD = 2  # if the same action is repeated more than this, apply penalty

    # Trigger Arduino reset via GPIO14
    print("Resetting Arduino via GPIO14...")
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(2)  # allow time for Arduino to reset and re-enumerate

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

    # --- Load Q-table ---
    q_table = load_q_table()

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
            parts = line.split(',')
            if len(parts) < 7:
                continue
            try:
                distances = [float(parts[i]) for i in range(6)]
                current_heading = float(parts[6])
            except Exception as e:
                print("Error parsing sensor data:", e)
                continue

            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # --- Obstacle Evaluation and Q-learning Modification ---
            front_sensors = distances[0:3]
            # Increase detection threshold: trigger if any front sensor < 30
            if any(d < 30 for d in front_sensors):
                print("Obstacle detected! Initiating ML pivot procedure.")
                left_clearance = front_sensors[0]
                right_clearance = front_sensors[2]
                print(f"Left clearance: {left_clearance}, Right clearance: {right_clearance}")
                # IMPORTANT: Now, if there is more space on the right, we want to pivot right.
                if right_clearance > left_clearance:
                    chosen_action = 1  # pivot right
                    print("Evaluation: More space on right. Choosing to pivot right.")
                else:
                    chosen_action = 0  # pivot left
                    print("Evaluation: More space on left. Choosing to pivot left.")

                if last_chosen_action is not None and chosen_action == last_chosen_action:
                    repeat_count += 1
                else:
                    repeat_count = 0
                last_chosen_action = chosen_action
                extra_penalty = -abs(right_clearance - left_clearance) if repeat_count > REPEAT_THRESHOLD else 0
                clearance_diff = abs(right_clearance - left_clearance)
                if (chosen_action == 0 and left_clearance >= right_clearance) or (chosen_action == 1 and right_clearance >= left_clearance):
                    reward = clearance_diff + extra_penalty
                else:
                    reward = -clearance_diff + extra_penalty
                state = get_state(front_sensors)
                new_state = state  # For simplicity, state remains the same
                update_q_value(q_table, state, chosen_action, reward, new_state)
                save_q_table(q_table)
                q_learning_step(chosen_action, duration=1.0)
                print("ML pivot complete. Resuming forward motion.")

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
                print(f"Heading: {current_heading:.2f}Â°, PID: {pid_output:.2f}, Left: {left_speed:.2f}%, Right: {right_speed:.2f}%")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Stopping the robot.")
    finally:
        stop_motors()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
