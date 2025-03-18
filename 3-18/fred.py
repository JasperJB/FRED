import subprocess
import time

def main():
    print("Starting motorcontrol.py...")
    motor_process = subprocess.Popen(["python3", "motorcontrol.py"])
    time.sleep(5)  # Ensure motorcontrol.py has time to start
    
    print("Starting control.py...")
    control_process = subprocess.Popen(["python3", "control.py"])
    
    print("Starting arduino_data.py...")
    arduino_process = subprocess.Popen(["python3", "arduino_data.py"])
    time.sleep(3)  # Ensure Arduino data collection is initialized
    
    print("Starting data_collection.py...")
    data_collection_process = subprocess.Popen(["python3", "data_collection.py"])
    
    try:
        motor_process.wait()
        control_process.wait()
        arduino_process.wait()
        data_collection_process.wait()
    except KeyboardInterrupt:
        print("Stopping all processes.")
        motor_process.terminate()
        control_process.terminate()
        arduino_process.terminate()
        data_collection_process.terminate()
        motor_process.wait()
        control_process.wait()
        arduino_process.wait()
        data_collection_process.wait()

if __name__ == "__main__":
    main()
