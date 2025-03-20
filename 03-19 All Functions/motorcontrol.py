import RPi.GPIO as GPIO
import redis
import logging
from rich.logging import RichHandler
from datetime import datetime
import os
import sys

MOTOR_PINS = {
    "ENA": 12,
    "ENB": 13,
    "IN1": 17,
    "IN2": 18,
    "IN3": 27,
    "IN4": 22,
}

# Initialize GPIO pins for motor driver
GPIO.setmode(GPIO.BCM)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)
GPIO.output(MOTOR_PINS["ENA"], GPIO.HIGH)
GPIO.output(MOTOR_PINS["ENB"], GPIO.HIGH)

def set_motor_state(in1, in2, in3, in4):
    """Activate motor driver outputs according to the provided bit states."""
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH if in1 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH if in2 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH if in3 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH if in4 else GPIO.LOW)

def main():
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    pubsub = r.pubsub()
    pubsub.subscribe("motor_command")
    logging.info("Motor control subscribed to Redis channel 'motor_command'")

    try:
        for message in pubsub.listen():
            if message["type"] != "message":
                continue
            cmd = message["data"].decode()  # e.g. "1,0,1,0"
            if not cmd:
                continue
            try:
                in1, in2, in3, in4 = map(int, cmd.split(","))
            except ValueError:
                logging.error(f"Invalid motor command format: '{cmd}'")
                continue
            set_motor_state(in1, in2, in3, in4)
            logging.debug(f"Executed motor command: {cmd}")
    except KeyboardInterrupt:
        logging.info("Shutting down motor control")
    except Exception as e:
        logging.exception("Unexpected error in motorcontrol")
        err_message = f"{type(e).__name__}: {e}"
        try:
            r.publish("errors", f"motorcontrol.py | {err_message}")
        except Exception:
            pass
        raise
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    # Determine debug mode from CLI flag or environment
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")

    # Configure logging to file and console (rich formatted)
    log_file = f"motorcontrol_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    try:
        main()
    except Exception as e:
        logging.exception("motorcontrol script terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"motorcontrol.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
