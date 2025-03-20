import redis
import ast
import numpy as np
import matplotlib.pyplot as plt
import logging
from rich.logging import RichHandler
from datetime import datetime
import os
import sys

def main():
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)
    plt.ion()
    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title("Rover Depth Visualization")
    # Initialize 9x12 heatmap with zeros
    data = np.zeros((9, 12), dtype=np.float32)
    heatmap = ax.imshow(data, cmap="jet", vmin=0, vmax=1, origin="upper")
    cbar = plt.colorbar(heatmap, ax=ax)
    cbar.set_label("Normalized Distance (0..1)")
    ax.set_xticks(range(12))
    ax.set_yticks(range(9))
    ax.set_title("Heading: 0.00  |  Recording: Disabled")
    plt.tight_layout()

    try:
        while True:
            # 1) Depth map from camera
            camera_val = rdb.get("camera_depth")
            if camera_val is not None:
                try:
                    arr_list = ast.literal_eval(camera_val.decode())
                    arr = np.array(arr_list, dtype=np.float32)
                    if arr.shape == (9, 12):
                        heatmap.set_data(arr)
                except Exception:
                    # Ignore malformed depth data
                    pass
            # 2) Current heading
            heading_val = rdb.get("arduino_heading")
            heading = 0.0
            if heading_val is not None:
                try:
                    heading = float(heading_val.decode())
                except Exception:
                    pass
            # 3) Recording status
            rec_val = rdb.get("recording_enabled")
            recording_enabled = False
            if rec_val is not None:
                try:
                    recording_enabled = (int(rec_val.decode()) == 1)
                except Exception:
                    pass
            # Update the plot title with current values
            ax.set_title(f"Heading: {heading:.2f}  |  Recording: {'Enabled' if recording_enabled else 'Disabled'}")
            plt.draw()
            plt.pause(0.1)
    except KeyboardInterrupt:
        logging.info("Closing data_visualizer...")
    except Exception as e:
        logging.exception("Unexpected error in data_visualizer")
        err_message = f"{type(e).__name__}: {e}"
        try:
            rdb.publish("errors", f"data_visualizer.py | {err_message}")
        except Exception:
            pass
        raise
    finally:
        plt.ioff()
        plt.close(fig)

if __name__ == "__main__":
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")
    log_file = f"data_visualizer_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    try:
        main()
    except Exception as e:
        logging.exception("data_visualizer script terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"data_visualizer.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
