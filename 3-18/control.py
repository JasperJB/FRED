import pygame
import socket
import time

def detect_controller():
    pygame.init()
    pygame.joystick.init()
    
    if pygame.joystick.get_count() == 0:
        print("No controller detected. Please connect a PS4 controller.")
        return None
    
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected controller: {joystick.get_name()}")
    return joystick

def send_motor_command(sock, in1, in2, in3, in4):
    command = f"{in1}{in2}{in3}{in4}"
    sock.sendall(command.encode())

def send_reset_command():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 5001))  # Connect to arduino_data.py reset listener
        sock.sendall(b'reset')
        sock.close()
        print("?? Sent reset command to Arduino!")
    except ConnectionRefusedError:
        print("? Failed to send reset command: Arduino reset server not running.")

def connect_to_motor_control():
    server_address = ('127.0.0.1', 5000)  # Use IP instead of "localhost"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    for attempt in range(10):
        try:
            print(f"?? Attempt {attempt + 1}: Connecting to {server_address}...")
            sock.connect(server_address)
            print("? Successfully connected to motor control script")
            return sock
        except ConnectionRefusedError as e:
            print(f"? Connection attempt {attempt + 1} failed. Error: {e}")
            time.sleep(1)

    print("?? Failed to connect to motor control script after 10 attempts. Exiting.")
    return None

def main():
    joystick = detect_controller()
    if not joystick:
        return
    
    sock = connect_to_motor_control()
    if not sock:
        return
    
    try:
        while True:
            pygame.event.pump()
            
            L1 = joystick.get_button(4)  # L1
            L2 = joystick.get_button(6)  # L2
            R1 = joystick.get_button(5)  # R1
            R2 = joystick.get_button(7)  # R2
            Triangle = joystick.get_button(2)  # Triangle button for Arduino reset
            
            if Triangle:
                print("?? Triangle button pressed: Sending reset command...")
                send_reset_command()
                time.sleep(0.5)  # Prevent rapid firing of reset command
            
            # Left and right motors are reversed
            left_motor_1 = R1
            left_motor_2 = R2
            right_motor_1 = L1
            right_motor_2 = L2
            
            send_motor_command(sock, left_motor_1, left_motor_2, right_motor_1, right_motor_2)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping control script.")
    finally:
        sock.close()
        pygame.quit()

if __name__ == "__main__":
    main()
