#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import serial
from serial.tools import list_ports

def find_arduino():
    """Scan serial ports for an Arduino device."""
    for port in list_ports.comports():
        # Check common Arduino port names (ttyACM* or ttyUSB*)
        if "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    return None

def main():
    RESET_PIN = 14  # GPIO14 (using BCM numbering)
    
    # Configure GPIO for resetting the Arduino
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RESET_PIN, GPIO.OUT)
    
    # Reset the Arduino by setting GPIO14 high briefly
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.LOW)
    # Allow time for the Arduino to begin rebooting
    time.sleep(1)
    
    # Attempt to detect the Arduino on available serial ports for up to 5 seconds
    port = None
    timeout = 5  # seconds
    start_time = time.time()
    while port is None and (time.time() - start_time) < timeout:
        port = find_arduino()
        if port is None:
            time.sleep(0.5)
    
    if port is None:
        print("Arduino not found on any serial port within the timeout period.")
        GPIO.cleanup()
        return

    print("Arduino found on port:", port)
    
    # Wait 2 seconds to ensure the Arduino reboot is fully complete
    time.sleep(2)
    
    # Open the serial connection to the Arduino
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)  # Allow serial connection to stabilize
    
    try:
        while True:
            # Read a line from the Arduino and print it
            line = ser.readline().decode('utf-8').strip()
            if line:
                print("Heading:", line)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
