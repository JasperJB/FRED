import pygame
import redis
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

def main():
    joystick = detect_controller()
    if not joystick:
        return

    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    
    print("Control script publishing to 'motor_command' and 'arduino_reset' channels...")
    print("Also updating 'recording_enabled' in Redis via X/O buttons...")
    
    try:
        while True:
            pygame.event.pump()
            
            # Common PS4 button indices:
            #   X (Cross) = 0
            #   O (Circle) = 1
            #   Triangle = 2
            #   Square = 3
            #   L1 = 4, R1 = 5, L2 = 6, R2 = 7, etc.
            
            cross = joystick.get_button(0)     # X
            circle = joystick.get_button(1)    # O
            triangle = joystick.get_button(2)  # Triangle for Arduino reset
            l1 = joystick.get_button(4)        # L1
            r1 = joystick.get_button(5)        # R1
            l2 = joystick.get_button(6)        # L2
            r2 = joystick.get_button(7)        # R2
            
            # 1) Reset Arduino when Triangle is pressed
            if triangle:
                print("Triangle pressed: Publishing reset command...")
                r.publish("arduino_reset", "reset")
                time.sleep(0.5)
            
            # 2) Toggle recording state
            if circle:  # O â†’ Enable recording
                print("Circle (O) pressed: recording enabled")
                r.set("recording_enabled", 1)
                time.sleep(0.2)
            if cross:   # X â†’ Disable recording
                print("Cross (X) pressed: recording disabled")
                r.set("recording_enabled", 0)
                time.sleep(0.2)
            
            # 3) Determine motor bits
            # Left and right motors are reversed as before
            left_motor_1 = r1
            left_motor_2 = r2
            right_motor_1 = l1
            right_motor_2 = l2
            
            cmd = f"{left_motor_1},{left_motor_2},{right_motor_1},{right_motor_2}"
            r.publish("motor_command", cmd)
            r.set("motor_state", cmd)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping control script.")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
