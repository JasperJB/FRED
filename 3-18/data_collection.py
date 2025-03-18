import csv
import time
import socket
from datetime import datetime

def get_arduino_data():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(('localhost', 5002))  # Assuming arduino_data.py serves data on port 5002
            sock.sendall(b'get')
            data = sock.recv(1024).decode()
            return float(data)
    except ConnectionRefusedError:
        return None

def get_motor_data():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect(('localhost', 5003))  # Assuming control.py provides data on port 5003
            sock.sendall(b'get')
            data = sock.recv(1024).decode()
            return data.strip()
    except ConnectionRefusedError:
        return None

def main():
    filename = f"data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    print(f"Saving data to {filename}")
    
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Heading (0-1)", "Motor State"])
        
        try:
            while True:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                heading = get_arduino_data()
                motor_state = get_motor_data()
                
                if heading is not None and motor_state is not None:
                    writer.writerow([timestamp, heading, motor_state])
                    print(f"Logged: {timestamp}, {heading}, {motor_state}")
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopping data collection.")

if __name__ == "__main__":
    main()
