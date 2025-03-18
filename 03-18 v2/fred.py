import subprocess
import time

def main():
    print("Starting motorcontrol.py with Redis...")
    motor_process = subprocess.Popen(["python3", "motorcontrol.py"])
    time.sleep(5)  # Ensure motorcontrol.py has time to start
    
    print("Starting control.py with Redis...")
    control_process = subprocess.Popen(["python3", "control.py"])
    
    print("Starting arduino_data.py with Redis...")
    arduino_process = subprocess.Popen(["python3", "arduino_data.py"])
    time.sleep(3)  # Ensure Arduino data collection is initialized
    
    # NEW: Start camera_data.py
    print("Starting camera_data.py with Redis...")
    camera_process = subprocess.Popen(["python3", "camera_data.py"])
    
    print("Starting data_collection.py with Redis...")
    data_collection_process = subprocess.Popen(["python3", "data_collection.py"])
    
    try:
        motor_process.wait()
        control_process.wait()
        arduino_process.wait()
        camera_process.wait()  # Wait for camera_data.py
        data_collection_process.wait()
    except KeyboardInterrupt:
        print("Stopping all processes.")
        motor_process.terminate()
        control_process.terminate()
        arduino_process.terminate()
        camera_process.terminate()
        data_collection_process.terminate()
        motor_process.wait()
        control_process.wait()
        arduino_process.wait()
        camera_process.wait()
        data_collection_process.wait()

if __name__ == "__main__":
    main()
