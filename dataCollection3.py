import pygame
import RPi.GPIO as GPIO
import time
import serial
import serial.tools.list_ports
import threading
import csv
import numpy as np
import ArducamDepthCamera as ac

# ---------------- GPIO & Motor Setup ----------------
RESET_PIN = 14
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor Pins
MOTOR_PINS = {"ENA": 12, "ENB": 13, "IN1": 17, "IN2": 18, "IN3": 27, "IN4": 22}
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)
GPIO.setup(RESET_PIN, GPIO.OUT)

ENA_PWM = GPIO.PWM(MOTOR_PINS["ENA"], 1000)
ENB_PWM = GPIO.PWM(MOTOR_PINS["ENB"], 1000)
ENA_PWM.start(0)
ENB_PWM.start(0)

# ---------------- Motor Control ----------------
def set_motor(left_speed, right_speed):
    GPIO.output(MOTOR_PINS["IN1"], left_speed >= 0)
    GPIO.output(MOTOR_PINS["IN2"], left_speed < 0)
    GPIO.output(MOTOR_PINS["IN3"], right_speed >= 0)
    GPIO.output(MOTOR_PINS["IN4"], right_speed < 0)
    ENA_PWM.ChangeDutyCycle(min(abs(left_speed), 100))
    ENB_PWM.ChangeDutyCycle(min(abs(right_speed), 100))

def stop_motors():
    ENA_PWM.ChangeDutyCycle(0)
    ENB_PWM.ChangeDutyCycle(0)
    for pin in ["IN1", "IN2", "IN3", "IN4"]:
        GPIO.output(MOTOR_PINS[pin], GPIO.LOW)

# ---------------- Arduino + Camera Setup ----------------
def reset_arduino():
    print("Resetting Arduino via GPIO14...")
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(RESET_PIN, GPIO.LOW)
    print("Waiting for Arduino to reboot...")
    time.sleep(2)  # Same as nanoComms.py

def find_arduino():
    print("Searching for Arduino...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "USB" in port.device or "ttyUSB" in port.device or "ttyACM" in port.device:
            print(f"Arduino found on port: {port.device}")
            return port.device
    return None

def setup_camera():
    cam = ac.ArducamCamera()
    if cam.open(ac.Connection.CSI, 0) != 0 or cam.start(ac.FrameType.DEPTH) != 0:
        raise RuntimeError("Failed to initialize camera")
    return cam

def downsample_depth(depth_data, target_shape=(24, 18)):
    return depth_data.reshape(target_shape[1], depth_data.shape[0] // target_shape[1],
                              target_shape[0], depth_data.shape[1] // target_shape[0]).mean(axis=(1, 3))

# ---------------- Data Collection Threads ----------------
def collect_arduino_data(arduino, shared_data, stop_event):
    while not stop_event.is_set():
        line = arduino.readline().decode().strip()
        if line:
            values = line.split(',')
            if len(values) == 5:  # Expecting 5 values, like nanoComms
                try:
                    dist_values = list(map(float, values))
                    shared_data["arduino"] = dist_values
                except ValueError:
                    print(f"Invalid data received: {line}")

def collect_camera_data(cam, shared_data, stop_event):
    while not stop_event.is_set():
        frame = cam.requestFrame(2000)
        if frame and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            depth_small = downsample_depth(depth_buf)
            shared_data["camera"] = depth_small.flatten().tolist()
            cam.releaseFrame(frame)

# ---------------- Main Control Loop ----------------
def main():
    # Reset Arduino and detect it
    reset_arduino()

    arduino_port = None
    while not arduino_port:
        arduino_port = find_arduino()
        if not arduino_port:
            print("Retrying Arduino connection...")
            time.sleep(2)

    arduino = serial.Serial(arduino_port, 115200, timeout=1)
    time.sleep(2)  # Ensure Arduino is fully ready

    print("Arduino connected.")

    # Setup Arducam
    cam = setup_camera()
    print("Camera initialized.")

    # Setup PS4 controller
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"PS4 Controller connected: {joystick.get_name()}")
    DEADZONE = 0.2

    # Data collection setup
    collecting_data = False
    shared_data = {"arduino": [], "camera": []}
    stop_event = threading.Event()

    from datetime import datetime  # Add this import at the top with others

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"tensorflow_data_{timestamp}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    print(f"Saving data to {csv_filename}")
    writer = csv.writer(csv_file)
    cam_headers = [f"Sector_{i}" for i in range(24 * 18)]
    arduino_headers = ['Dist1', 'Dist2', 'Dist3', 'Dist4', 'Heading']
    motor_headers = ['LeftMotor', 'RightMotor']
    writer.writerow(cam_headers + arduino_headers + motor_headers)

    # Start threads
    arduino_thread = threading.Thread(target=collect_arduino_data, args=(arduino, shared_data, stop_event))
    camera_thread = threading.Thread(target=collect_camera_data, args=(cam, shared_data, stop_event))
    arduino_thread.start()
    camera_thread.start()

    try:
        while True:
            pygame.event.pump()
            left_axis = -joystick.get_axis(1)
            right_axis = -joystick.get_axis(4)

            # Deadzone
            left_axis = left_axis if abs(left_axis) > DEADZONE else 0
            right_axis = right_axis if abs(right_axis) > DEADZONE else 0

            left_speed = int(left_axis * 100)
            right_speed = int(right_axis * 100)
            set_motor(right_speed, left_speed)

            # Button handling
            if joystick.get_button(1):  # O
                print("Started collecting data")
                collecting_data = True
            if joystick.get_button(0):  # X
                print("Stopped collecting data")
                collecting_data = False

            # Collect if moving & active
            if collecting_data and (left_speed != 0 or right_speed != 0):
                if shared_data["arduino"] and shared_data["camera"]:
                    row = shared_data["camera"] + shared_data["arduino"] + [left_speed, right_speed]
                    writer.writerow(row)
                    print(f"Data row written: {row[:5]}...")

            time.sleep(0.05)  # 20 Hz

    except KeyboardInterrupt:
        print("Keyboard interrupt, stopping...")

    finally:
        stop_event.set()
        arduino_thread.join()
        camera_thread.join()
        csv_file.close()
        stop_motors()
        ENA_PWM.stop()
        ENB_PWM.stop()
        GPIO.cleanup()
        pygame.quit()
        cam.stop()
        cam.close()

if __name__ == "__main__":
    main()
