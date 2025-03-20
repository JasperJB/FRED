import pygame
import redis
import time
import logging
from rich.logging import RichHandler
from datetime import datetime
import os
import sys

def detect_controller():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        return None
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    logging.info(f"Detected controller: {joystick.get_name()}")
    return joystick

def main():
    joystick = detect_controller()
    if not joystick:
        # No controller found – log error and notify via Redis, then exit
        logging.error("No controller detected. Please connect a PS4 controller.")
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", "control.py | No PS4 controller detected")
        except Exception:
            pass
        return

    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    logging.info("Control script publishing to 'motor_command' and 'arduino_reset' channels")
    logging.info("Using PS4 controller buttons: O/X to toggle recording, Triangle to reset Arduino")

    try:
        while True:
            pygame.event.pump()
            # Read PS4 button states
            cross   = joystick.get_button(0)   # X (Cross)
            circle  = joystick.get_button(1)   # O (Circle)
            triangle= joystick.get_button(2)   # Triangle
            l1 = joystick.get_button(4); r1 = joystick.get_button(5)
            l2 = joystick.get_button(6); r2 = joystick.get_button(7)

            # 1) Reset Arduino when Triangle is pressed
            if triangle:
                logging.info("Triangle pressed: publishing reset command")
                r.publish("arduino_reset", "reset")
                time.sleep(0.5)

            # 2) Toggle recording state (Circle = start, Cross = stop)
            if circle:
                logging.info("Circle (O) pressed: recording enabled")
                r.set("recording_enabled", 1)
                time.sleep(0.2)
            if cross:
                logging.info("Cross (X) pressed: recording disabled")
                r.set("recording_enabled", 0)
                time.sleep(0.2)

            # 3) Publish motor command based on shoulder button inputs
            left_motor_1, left_motor_2 = r1, r2   # Note: left motors use R1/R2
            right_motor_1, right_motor_2 = l1, l2 # right motors use L1/L2
            cmd = f"{left_motor_1},{left_motor_2},{right_motor_1},{right_motor_2}"
            r.publish("motor_command", cmd)
            r.set("motor_state", cmd)
            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("Stopping control script")
    except Exception as e:
        logging.exception("Unexpected error in control script")
        err_message = f"{type(e).__name__}: {e}"
        try:
            r.publish("errors", f"control.py | {err_message}")
        except Exception:
            pass
        raise
    finally:
        pygame.quit()

if __name__ == "__main__":
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")
    log_file = f"control_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    try:
        main()
    except Exception as e:
        logging.exception("control script terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"control.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
