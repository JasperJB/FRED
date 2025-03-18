import serial
import time
import RPi.GPIO as GPIO
import socket
import glob

def reset_arduino():
    GPIO.output(14, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(14, GPIO.LOW)
    time.sleep(2)

def listen_for_reset_command():
    server_address = ('localhost', 5001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(server_address)
    sock.listen(1)
    print("Listening for reset commands...")
    
    while True:
        conn, _ = sock.accept()
        data = conn.recv(1024)
        if data.decode() == 'reset':
            print("Reset command received. Restarting Arduino...")
            reset_arduino()
        conn.close()

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
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14, GPIO.OUT)
    
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
                data = ser.readline().decode().strip()
                if data:
                    try:
                        heading = float(data)
                        scaled_heading = heading / 360.0
                        print(f"Heading: {heading} | Scaled: {scaled_heading:.2f}")
                    except ValueError:
                        print("Received invalid data from Arduino, skipping...")
        except (serial.SerialException, ValueError):
            print("Connection lost. Reconnecting...")
            time.sleep(2)
        
if __name__ == "__main__":
    from threading import Thread
    Thread(target=listen_for_reset_command, daemon=True).start()
    main()
