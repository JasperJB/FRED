# -*- coding: utf-8 -*-
import csv
import time
import redis
import math
import ast
from datetime import datetime

def get_arduino_data(r):
    val = r.get("arduino_heading")
    if val is None:
        return None
    try:
        return float(val)
    except ValueError:
        return None

def get_motor_data(r):
    val = r.get("motor_state")
    if val is None:
        return None
    return val.decode().strip()

def get_camera_data(r):
    val = r.get("camera_depth")
    if val is None:
        return None
    try:
        data = ast.literal_eval(val.decode())
        return data  # data is a 9x12 list-of-lists (floats)
    except:
        return None

def is_recording_enabled(r):
    val = r.get("recording_enabled")
    if val is None:
        return False
    return int(val) == 1

def main():
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    filename = f"data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    print(f"Saving data to {filename}")
    
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Heading (0-1)", "Motor State", "Camera Depth 9x12"])
        
        logged_count = 0
        try:
            while True:
                if not is_recording_enabled(r):
                    # No logging until recording is enabled
                    time.sleep(0.1)
                    continue
                
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                heading = get_arduino_data(r)
                motor_state = get_motor_data(r)
                camera_data = get_camera_data(r)

                if heading is not None and motor_state is not None and camera_data is not None:
                    # Only log if the rover is "moving"
                    if motor_state != "0,0,0,0":
                        # Truncate heading to 4 decimals
                        truncated_heading = math.floor(heading * 10000) / 10000
                        heading_str = f"{truncated_heading:.4f}"

                        # We'll store the camera data as a single string
                        camera_str = repr(camera_data)

                        writer.writerow([timestamp, heading_str, motor_state, camera_str])
                        logged_count += 1
                        
                        # Overwrite a single line showing how many rows we've logged
                        print(f"\rRows logged: {logged_count}", end="", flush=True)
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping data collection.")

if __name__ == "__main__":
    main()
