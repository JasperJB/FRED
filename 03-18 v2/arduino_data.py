# -*- coding: utf-8 -*-
import serial
import time
import RPi.GPIO as GPIO
import glob
import redis
from threading import Thread

CONFIDENCE_THRESHOLD = 30

def reset_arduino():
    GPIO.output(14, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(14, GPIO.LOW)
    time.sleep(2)

def handle_reset_commands():
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    pubsub = r.pubsub()
    pubsub.subscribe("arduino_reset")
    # Print one message, not repeated
    print("Listening for reset commands via Redis...")

    for message in pubsub.listen():
        if message["type"] == "message":
            if message["data"].decode() == 'reset':
                print("\nReset command received. Restarting Arduino...")
                reset_arduino()

def find_arduino():
    usb_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    for port in usb_ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            print(f"Arduino detected on {port}")
            return ser
        except serial.SerialException:
            continue
    return None

def main():
    # To avoid "This channel is already in use" warnings:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14, GPIO.OUT)
    
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)

    while True:
        print("Searching for Arduino...")
        ser = find_arduino()
        
        if ser is None:
            print("No Arduino found. Retrying in 2 seconds...")
            time.sleep(2)
            continue
        
        print("Data link established.")
        
        try:
            while True:
                data = ser.readline().decode(errors='ignore').strip()
                if data:
                    try:
                        heading = float(data)
                        scaled_heading = heading / 360.0
                        # Instead of printing a new line for every reading,
                        # overwrite the same line:
                        print(f"\rHeading: {heading:.2f} | Scaled: {scaled_heading:.2f}", end="", flush=True)
                        
                        # Store scaled heading in Redis for data_collection
                        r.set("arduino_heading", scaled_heading)
                    except ValueError:
                        # If line can't parse, do nothing
                        pass
        except (serial.SerialException, ValueError):
            print("\nConnection lost. Reconnecting...")
            time.sleep(2)

if __name__ == "__main__":
    Thread(target=handle_reset_commands, daemon=True).start()
    main()
