import serial
import time
import RPi.GPIO as GPIO
import glob
import redis
from threading import Thread
from collections import deque
import logging
from rich.logging import RichHandler
from datetime import datetime
import os
import sys

CONFIDENCE_THRESHOLD = 30      # Minimum valid confidence for distance readings
ROLLING_WINDOW_SIZE = 5        # Rolling average window size for distances

def reset_arduino():
    # Toggle the Arduino's reset pin (BCM GPIO 14)
    GPIO.output(14, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(14, GPIO.LOW)
    time.sleep(2)

def handle_reset_commands():
    """Background thread: listens for 'arduino_reset' commands to reset the Arduino."""
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    pubsub = r.pubsub()
    pubsub.subscribe("arduino_reset")
    logging.info("Listening for reset commands via Redis...")
    for message in pubsub.listen():
        try:
            if message["type"] == "message" and message["data"].decode() == 'reset':
                logging.info("Reset command received. Restarting Arduino...")
                reset_arduino()
        except Exception as e:
            logging.exception("Error in Arduino reset command handler")
            err_message = f"{type(e).__name__}: {e}"
            try:
                r.publish("errors", f"arduino_data.py | {err_message}")
            except Exception:
                pass
            break

def find_arduino():
    """Scan for an Arduino on any /dev/ttyUSB* or /dev/ttyACM* port."""
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
            logging.info(f"Arduino detected on {port}")
            return ser
        except serial.SerialException:
            continue
    return None

def rolling_average(deq, new_value):
    if len(deq) >= ROLLING_WINDOW_SIZE:
        deq.popleft()
    deq.append(new_value)
    return sum(deq) / len(deq)

def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(14, GPIO.OUT)  # Arduino reset pin setup

    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)
    # Buffers to compute rolling average of each distance sensor
    dist1_values = deque(maxlen=ROLLING_WINDOW_SIZE)
    dist2_values = deque(maxlen=ROLLING_WINDOW_SIZE)
    dist3_values = deque(maxlen=ROLLING_WINDOW_SIZE)
    dist4_values = deque(maxlen=ROLLING_WINDOW_SIZE)

    try:
        while True:
            logging.info("Searching for Arduino...")
            ser = find_arduino()
            if ser is None:
                logging.warning("No Arduino found. Retrying in 2 seconds...")
                time.sleep(2)
                continue

            logging.info("Data link established")
            try:
                while True:
                    line = ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    try:
                        # Expected format: "heading,dist1,dist2,dist3,dist4"
                        heading, d1, d2, d3, d4 = map(float, line.split(','))
                    except ValueError:
                        # Skip malformed line
                        continue
                    # Compute rolling averages for distance readings
                    avg_d1 = rolling_average(dist1_values, d1)
                    avg_d2 = rolling_average(dist2_values, d2)
                    avg_d3 = rolling_average(dist3_values, d3)
                    avg_d4 = rolling_average(dist4_values, d4)
                    # Scale heading to 0–1
                    scaled_heading = heading / 360.0
                    # Log sensor data in debug mode
                    logging.debug(f"Heading: {heading:.2f} | Dist1: {avg_d1:.2f} cm | "
                                  f"Dist2: {avg_d2:.2f} cm | Dist3: {avg_d3:.2f} cm | Dist4: {avg_d4:.2f} cm")
                    # Store latest values in Redis
                    rdb.set("arduino_heading", scaled_heading)
                    rdb.set("arduino_dist1", avg_d1)
                    rdb.set("arduino_dist2", avg_d2)
                    rdb.set("arduino_dist3", avg_d3)
                    rdb.set("arduino_dist4", avg_d4)
            except (serial.SerialException, ValueError):
                logging.warning("Connection lost. Reconnecting...")
                time.sleep(2)
                # go back to find_arduino loop
                continue
    except KeyboardInterrupt:
        logging.info("Stopping Arduino data script")
    except Exception as e:
        logging.exception("Unexpected error in arduino_data")
        err_message = f"{type(e).__name__}: {e}"
        try:
            rdb.publish("errors", f"arduino_data.py | {err_message}")
        except Exception:
            pass
        raise
    finally:
        try:
            ser.close()
        except Exception:
            pass
        GPIO.cleanup()

if __name__ == "__main__":
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")
    log_file = f"arduino_data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    # Start thread to handle Arduino reset commands
    Thread(target=handle_reset_commands, daemon=True).start()
    try:
        main()
    except Exception as e:
        logging.exception("arduino_data script terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"arduino_data.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
