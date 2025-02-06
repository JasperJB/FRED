import json
import serial
import time
import numpy as np
import random
from rich.progress import Progress
from pykalman import KalmanFilter
import RPi.GPIO as GPIO

# Serial port settings
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
CALIBRATION_SAMPLES = 20
Q_TABLE_FILE = "q_table.json"

# GPIO pin settings
MOTOR_PINS = {"ENA": 12, "ENB": 13, "IN1": 17, "IN2": 18, "IN3": 27, "IN4": 22}

# Q-Learning parameters
LEARNING_RATE = 0.2
DISCOUNT_FACTOR = 0.9
EXPLORATION_RATE = 0.3
MIN_EXPLORATION = 0.01
EXPLORATION_DECAY = 0.99
Q_TABLE = {}

ACTIONS = ["FORWARD", "LEFT", "RIGHT", "STOP"]

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# **CALIBRATION**
def collect_calibration_data():
    print("\n[INFO] Collecting calibration data...")
    imu_data = []
    with Progress() as progress:
        task = progress.add_task("[cyan]Calibrating...", total=CALIBRATION_SAMPLES)
        for _ in range(CALIBRATION_SAMPLES):
            line = ser.readline().decode("utf-8").strip()
            if line.startswith("IMU:"):
                try:
                    imu_values = list(map(float, line.split(";")[0][4:].split(",")))
                    imu_data.append(imu_values)
                except ValueError:
                    continue
            progress.advance(task, 1)
            time.sleep(0.05)

    imu_array = np.array(imu_data)
    imu_offsets = np.mean(imu_array, axis=0)
    print("\n[INFO] Calibration Complete!")
    print(f"IMU Offsets: {imu_offsets}")
    return imu_offsets

# **KALMAN FILTER**
def initialize_kalman_filter():
    return KalmanFilter(
        transition_matrices=np.eye(6),
        observation_matrices=np.eye(6),
        initial_state_mean=np.zeros(6),
        observation_covariance=np.eye(6) * 0.1,
        transition_covariance=np.eye(6) * 0.01,
    )

# **GPIO Setup**
def setup_motor_pins():
    GPIO.setmode(GPIO.BCM)
    for pin in MOTOR_PINS.values():
        GPIO.setup(pin, GPIO.OUT)
    pwm_right = GPIO.PWM(MOTOR_PINS["ENA"], 100)
    pwm_left = GPIO.PWM(MOTOR_PINS["ENB"], 100)
    pwm_right.start(0)
    pwm_left.start(0)
    return pwm_right, pwm_left

# **Motor Control**
def drive_forward(pwm_right, pwm_left, speed=50):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)

def turn_left(pwm_right, pwm_left, speed=50):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)

def turn_right(pwm_right, pwm_left, speed=50):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)
    pwm_right.ChangeDutyCycle(speed)
    pwm_left.ChangeDutyCycle(speed)

def stop(pwm_right, pwm_left):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    pwm_right.ChangeDutyCycle(0)
    pwm_left.ChangeDutyCycle(0)

# **Q-Learning**
def load_q_table():
    global Q_TABLE
    try:
        with open(Q_TABLE_FILE, "r") as f:
            Q_TABLE = json.load(f)
        print("[INFO] Loaded Q-table from file.")
    except (FileNotFoundError, json.JSONDecodeError):
        print("[INFO] No Q-table found. Creating a new one.")
        Q_TABLE = {}

def save_q_table():
    with open(Q_TABLE_FILE, "w") as f:
        json.dump(Q_TABLE, f, indent=4)
    print("[INFO] Q-table saved.")

def get_state(sensor_data):
    return tuple("near" if d < 30 else "medium" if d < 70 else "far" for d in sensor_data)

def choose_action(state):
    global EXPLORATION_RATE
    if state not in Q_TABLE:
        Q_TABLE[state] = {action: 0 for action in ACTIONS}
    if random.random() < EXPLORATION_RATE:
        return random.choice(ACTIONS)
    return max(Q_TABLE[state], key=lambda action: Q_TABLE[state].get(action, 0))

def get_reward(sensor_data, action):
    min_distance = min(sensor_data)
    if action == "FORWARD":
        return 50 if min_distance > 40 else -50
    elif action in ["LEFT", "RIGHT"]:
        return 10 if min_distance < 30 else -5
    elif action == "STOP":
        return -20
    return -10

# **Main Process**
if __name__ == "__main__":
    pwm_right, pwm_left = setup_motor_pins()
    load_q_table()
    imu_offsets = collect_calibration_data()
    kf = initialize_kalman_filter()
    prev_state_mean = np.zeros(6)
    prev_covariance = np.eye(6) * 0.1

    try:
        state = None
        while True:
            line = ser.readline().decode("utf-8").strip()
            if "IMU:" in line and "SONAR:" in line:
                parts = line.split(";")
                imu_data = list(map(float, parts[0][4:].strip().split(",")))
                raw_sonar_data = list(map(float, parts[1][7:].strip().split(",")))

                filtered_imu_data, prev_covariance = kf.filter_update(prev_state_mean, prev_covariance, observation=imu_data)
                prev_state_mean = filtered_imu_data

                next_state = get_state(raw_sonar_data)
                action = choose_action(next_state)

                # **Drive the Rover**
                if action == "FORWARD":
                    drive_forward(pwm_right, pwm_left, speed=50)
                elif action == "LEFT":
                    turn_left(pwm_right, pwm_left, speed=50)
                elif action == "RIGHT":
                    turn_right(pwm_right, pwm_left, speed=50)
                elif action == "STOP":
                    stop(pwm_right, pwm_left)

                reward = get_reward(raw_sonar_data, action)
                if state is not None:
                    Q_TABLE.setdefault(state, {})[action] = (1 - LEARNING_RATE) * Q_TABLE[state].get(action, 0) + LEARNING_RATE * (reward + DISCOUNT_FACTOR * max(Q_TABLE.get(next_state, {}).values(), default=0))
                state = next_state

                print(f"State: {state}, Action: {action}, Reward: {reward}")
                print(f"Raw Sonar Data: {raw_sonar_data}")
                print(f"Filtered IMU Data: {filtered_imu_data.round(3)}\n")

    except KeyboardInterrupt:
        print("\n[INFO] Exiting gracefully...")
    finally:
        save_q_table()
        stop(pwm_right, pwm_left)
        GPIO.cleanup()
        ser.close()
