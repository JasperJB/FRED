import RPi.GPIO as GPIO
import socket

# GPIO pin settings for motor control
MOTOR_PINS = {
    "ENA": 12,
    "ENB": 13,
    "IN1": 17,
    "IN2": 18,
    "IN3": 27,
    "IN4": 22,
}

GPIO.setmode(GPIO.BCM)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)
GPIO.output(MOTOR_PINS["ENA"], GPIO.HIGH)
GPIO.output(MOTOR_PINS["ENB"], GPIO.HIGH)

def set_motor_state(in1, in2, in3, in4):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH if in1 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH if in2 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH if in3 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH if in4 else GPIO.LOW)

def start_server():
    server_address = ('localhost', 5000)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(server_address)
    sock.listen(1)
    print("Motor control server listening...")
    
    conn, _ = sock.accept()
    print("Connection established with control script")
    print("Client connected to motor control server")    
    try:
        while True:
            data = conn.recv(4)
            if not data:
                break
            
            in1, in2, in3, in4 = map(int, data.decode().strip())
            set_motor_state(in1, in2, in3, in4)
    except KeyboardInterrupt:
        print("Shutting down motor control server.")
    finally:
        conn.close()
        sock.close()
        GPIO.cleanup()

if __name__ == "__main__":
    start_server()
