# -*- coding: utf-8 -*-
import csv
import time
import redis
import math
import ast
from datetime import datetime

def get_arduino_data(r, key):
    val = r.get(key)
    if val is None:
        return None
    try:
        return float(val)
    except ValueError:
        return None

def main():
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    filename = f"data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    print(f"Saving data to {filename}")
    
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        # CSV header includes timestamp, heading, 4 distances, 9x12 depth values, and 4 motor bits
        header = ["Timestamp", "Heading (0-1)", "Dist1 (cm)", "Dist2 (cm)", "Dist3 (cm)", "Dist4 (cm)"]
        for i in range(9):
            for j in range(12):
                header.append(f"Depth_{i}_{j} (0-1)")
        header += ["Motor_IN1", "Motor_IN2", "Motor_IN3", "Motor_IN4"]
        writer.writerow(header)

        logged_count = 0
        try:
            while True:
                # Check if recording is enabled before logging
                rec_val = r.get("recording_enabled")
                recording = False
                if rec_val is not None:
                    try:
                        recording = int(rec_val.decode()) == 1
                    except ValueError:
                        recording = False
                if not recording:
                    time.sleep(0.1)
                    continue

                # Retrieve sensor readings and motor state
                heading = get_arduino_data(r, "arduino_heading")
                dist1 = get_arduino_data(r, "arduino_dist1")
                dist2 = get_arduino_data(r, "arduino_dist2")
                dist3 = get_arduino_data(r, "arduino_dist3")
                dist4 = get_arduino_data(r, "arduino_dist4")
                motor_state_bytes = r.get("motor_state")
                camera_depth_bytes = r.get("camera_depth")

                # Ensure all data is available
                if None in (heading, dist1, dist2, dist3, dist4) or motor_state_bytes is None or camera_depth_bytes is None:
                    time.sleep(0.1)
                    continue

                # Parse camera depth data (stored as a list-of-lists string in Redis)
                try:
                    camera_depth = ast.literal_eval(camera_depth_bytes.decode())
                except Exception:
                    camera_depth = None
                if camera_depth is None or not isinstance(camera_depth, list):
                    time.sleep(0.1)
                    continue
                depth_flat = [val for row in camera_depth for val in row]  # flatten 9x12 depth matrix

                # Parse motor state command into 4 bits
                motor_state_str = motor_state_bytes.decode()
                try:
                    motor_bits = list(map(int, motor_state_str.split(',')))
                except Exception:
                    motor_bits = None
                if motor_bits is None or len(motor_bits) != 4:
                    time.sleep(0.1)
                    continue

                # Truncate and format sensor values
                truncated_heading = math.floor(heading * 10000) / 10000  # 4 decimal places
                # Prepare a single row of data
                row = [
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    f"{truncated_heading:.4f}",
                    f"{dist1:.2f}", f"{dist2:.2f}", f"{dist3:.2f}", f"{dist4:.2f}"
                ]
                # Append depth values (formatted to 4 decimal places) and motor bits
                row += [f"{d:.4f}" for d in depth_flat]
                row += [str(bit) for bit in motor_bits]
                writer.writerow(row)

                logged_count += 1
                print(f"\rRows logged: {logged_count}", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopping data collection.")

if __name__ == "__main__":
    main()
