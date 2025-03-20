import ArducamDepthCamera as ac
import numpy as np
import redis
import time
import logging
from rich.logging import RichHandler
from datetime import datetime
import os
import sys

CONFIDENCE_THRESHOLD = 30
MAX_DISTANCE = 850
SRC_WIDTH, SRC_HEIGHT = 240, 180
DST_WIDTH, DST_HEIGHT = 12, 9

def open_camera():
    """Initialize the Arducam depth camera. Raises RuntimeError on failure."""
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        cam.close()
        raise RuntimeError(f"Failed to open depth camera (error code {ret})")
    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        cam.close()
        raise RuntimeError(f"Failed to start depth camera (error code {ret})")
    logging.info(f"Arducam Depth Camera started (SDK version {ac.__version__})")
    return cam

def block_average(depth_array):
    """Compute a 9x12 block average from a 180x240 depth array."""
    out = np.zeros((DST_HEIGHT, DST_WIDTH), dtype=np.float64)
    block_h = SRC_HEIGHT // DST_HEIGHT  # 20
    block_w = SRC_WIDTH // DST_WIDTH    # 20
    for row in range(DST_HEIGHT):
        for col in range(DST_WIDTH):
            start_h = row * block_h
            end_h   = start_h + block_h
            start_w = col * block_w
            end_w   = start_w + block_w
            block = depth_array[start_h:end_h, start_w:end_w]
            out[row, col] = np.mean(block)
    return out

def main():
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)
    cam = open_camera()
    info = cam.getCameraInfo()
    logging.info(f"Camera resolution: {info.width}x{info.height} (expected 240x180)")
    logging.info("camera_data.py is now streaming depth data to Redis")

    try:
        while True:
            frame = cam.requestFrame(2000)  # wait up to 2 seconds for a frame
            if frame is not None and isinstance(frame, ac.DepthData):
                depth_buf = frame.depth_data        # (180, 240) depth values
                conf_buf = frame.confidence_data   # confidence map
                # 1) Zero out low-confidence pixels
                depth_buf = np.where(conf_buf < CONFIDENCE_THRESHOLD, 0, depth_buf)
                # 2) Clamp values above MAX_DISTANCE
                depth_buf = np.clip(depth_buf, 0, MAX_DISTANCE)
                # 3) Downsample: compute 9x12 block averages
                small_depth = block_average(depth_buf)
                # 4) Normalize to [0,1]
                small_depth /= MAX_DISTANCE
                # 5) Truncate to 4 decimal places
                small_depth = np.floor(small_depth * 10000) / 10000
                # Store the 9x12 depth map in Redis as a string
                rdb.set("camera_depth", repr(small_depth.tolist()))
                cam.releaseFrame(frame)
            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("Stopping camera data script")
    except Exception as e:
        logging.exception("Unexpected error in camera_data")
        err_message = f"{type(e).__name__}: {e}"
        try:
            rdb.publish("errors", f"camera_data.py | {err_message}")
        except Exception:
            pass
        raise
    finally:
        cam.stop()
        cam.close()

if __name__ == "__main__":
    debug_mode = any(arg in ("--debug", "-d") for arg in sys.argv[1:]) or os.getenv("DEBUG", "").lower() in ("1", "true", "yes")
    log_file = f"camera_data_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
    logging.basicConfig(level=logging.DEBUG if debug_mode else logging.INFO,
                        format="%(message)s", datefmt="[%X]",
                        handlers=[logging.FileHandler(log_file, mode='w'),
                                  RichHandler(rich_tracebacks=True)])
    try:
        main()
    except Exception as e:
        logging.exception("camera_data script terminated due to an error")
        err_message = f"{type(e).__name__}: {e}"
        try:
            redis.Redis(host='127.0.0.1', port=6379, db=0).publish("errors", f"camera_data.py | {err_message}")
        except Exception:
            pass
        sys.exit(1)
