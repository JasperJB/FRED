import psutil
import csv
import time
from datetime import datetime
import logging
from rich.logging import RichHandler
import os
import sys

def main():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"cpu_memory_log_{timestamp}.csv"
    logging.info(f"Writing CPU/memory log to {filename}")
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "CPU_Usage(%)", "Memory_Usage(%)"])
        try:
            while True:
                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                # Sample CPU and RAM usage (pauses 1s each call)
                cpu_usage = psutil.cpu_percent(interval=1)
                memory_usage = psutil.virtual_memory().percent
                writer.writerow([current_time, cpu_usage, memory_usage])
                logging.info(f"{current_time} | CPU: {cpu_usage}% | Memory: {memory_usage}%")
        except KeyboardInterrupt:
            logging.info("Logging stopped")
        except Exception as e:
            logging.exception("Unexpected error in cpu_memory_logger")
            raise

if __name__ == "__main__":
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")
    log_file = f"cpu_memory_logger_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    try:
        main()
    except Exception as e:
        logging.exception("cpu_memory_logger terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            import redis
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"cpu_memory_logger.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
