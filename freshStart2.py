import evdev
from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
import time
import csv
import smbus  # for I2C

# ----------------- Motor Pin Setup -----------------
ENA = 12  # PWM for right motors (GPIO12 -> Physical pin 32)
ENB = 13  # PWM for left motors  (GPIO13 -> Physical pin 33)

IN1 = 17  # Right motor forward (GPIO17 -> Pin 11)
IN2 = 18  # Right motor reverse (GPIO18 -> Pin 12)
IN3 = 27  # Left motor forward  (GPIO27 -> Pin 13)
IN4 = 22  # Left motor reverse  (GPIO22 -> Pin 15)

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

pwm_right = GPIO.PWM(ENA, 1000)
pwm_left  = GPIO.PWM(ENB, 1000)
pwm_right.start(0)
pwm_left.start(0)

# ----------------- I2C Setup -----------------
I2C_ADDR = 0x10
bus = smbus.SMBus(1)  # On Pi 3B, I2C bus is often 1

# ---------------
# Helper function
# ---------------
def read_arduino_data():
    """
    Request sensor data from Arduino over I2C,
    which returns a comma-separated string.
    """
    # We'll assume the maximum length is ~32 bytes for SMBus block read.
    length = 32  
    raw_data = bus.read_i2c_block_data(I2C_ADDR, 0, length)
    # Convert raw_data (array of ints) into a string until we hit a 0 or end.
    chars = []
    for b in raw_data:
        if b == 0:
            break
        chars.append(chr(b))
    return "".join(chars)

# ----------------- CSV Logging -----------------
csv_file = open('robot_data.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["leftMotor", "rightMotor", "posX", "posY", 
                     "velocity", "heading", "dist1", "dist2", 
                     "dist3", "dist4", "dist5", "dist6"])

# ----------------- PS4 Controller Setup -----------------
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
ps4 = None
for dev in devices:
    if "Wireless Controller" in dev.name or "PS4" in dev.name or "Dualshock" in dev.name:
        ps4 = InputDevice(dev.path)
        break

if ps4 is None:
    print("No PS4 controller found. Make sure it’s paired via Bluetooth!")
    exit(1)

print(f"Using controller: {ps4.name} at {ps4.path}")

def stick_value(raw_val):
    # Scale joystick value [-32768..32767] to [-1..1]
    return raw_val / 32767.0

left_motor_command = 0.0
right_motor_command = 0.0

try:
    ps4.grab()  # lock device
    for event in ps4.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_Y:  
                # Left stick vertical
                left_stick_val = stick_value(event.value)  
                left_motor_command = -left_stick_val  # invert if needed
            elif event.code == ecodes.ABS_RZ: 
                # Right stick vertical
                right_stick_val = stick_value(event.value)
                right_motor_command = -right_stick_val

            # Convert to a speed 0..100 for PWM
            left_speed = abs(left_motor_command) * 100.0
            right_speed = abs(right_motor_command) * 100.0

            # Set direction pins
            # Left motors
            if left_motor_command >= 0:
                GPIO.output(IN3, GPIO.HIGH)
                GPIO.output(IN4, GPIO.LOW)
            else:
                GPIO.output(IN3, GPIO.LOW)
                GPIO.output(IN4, GPIO.HIGH)

            # Right motors
            if right_motor_command >= 0:
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
            else:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)

            # Update PWM
            pwm_left.ChangeDutyCycle(left_speed)
            pwm_right.ChangeDutyCycle(right_speed)

            # Now request data from Arduino & log it
            try:
                data = read_arduino_data()
                if data is not None:
                    # Debug print
                    print("Raw I2C Data from Arduino:", data)
                    fields = data.split(',')
                    # fields = [leftMotor, rightMotor, x, y, velocity, heading, dist1..dist6]
                    
                    # Overwrite the first two fields with the actual Pi-based motor commands
                    # if you want that in the CSV
                    fields[0] = f"{left_motor_command:.2f}"
                    fields[1] = f"{right_motor_command:.2f}"

                    if len(fields) == 12:
                        csv_writer.writerow(fields)
                        csv_file.flush()
                    else:
                        print("Unexpected number of CSV fields from Arduino:", len(fields))
            except Exception as e:
                print("I2C read error:", e)

except KeyboardInterrupt:
    print("Exiting due to Ctrl-C")
finally:
    ps4.ungrab()
    csv_file.close()
    pwm_right.stop()
    pwm_left.stop()
    GPIO.cleanup()
