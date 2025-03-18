import ArducamDepthCamera as ac
import numpy as np
import redis
import time
import math

CONFIDENCE_THRESHOLD = 30   # Remove low-confidence pixels
MAX_DISTANCE = 600          # Clamp distances above 600
SRC_WIDTH = 240
SRC_HEIGHT = 180
DST_WIDTH = 12
DST_HEIGHT = 9

def open_camera():
    """
    Initialize and open the Arducam depth camera using the CSI interface.
    Returns the camera object on success, or None on failure.
    """
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return None
    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return None
    print(f"Arducam Depth Camera started. SDK version: {ac.__version__}")
    return cam

def block_average(depth_array):
    """
    Given a 180Ã—240 array, compute a 9Ã—12 block average.
    Each cell in the output = mean of the corresponding 20Ã—20 block in the input.
    Returns a 9Ã—12 array of float64.
    """
    # Ensure we're in float64
    depth_array = depth_array.astype(np.float64, copy=False)

    out = np.zeros((DST_HEIGHT, DST_WIDTH), dtype=np.float64)
    block_h = SRC_HEIGHT // DST_HEIGHT  # 180//9 = 20
    block_w = SRC_WIDTH // DST_WIDTH    # 240//12=20

    for row in range(DST_HEIGHT):
        for col in range(DST_WIDTH):
            start_h = row * block_h
            end_h   = start_h + block_h
            start_w = col * block_w
            end_w   = start_w + block_w

            block = depth_array[start_h:end_h, start_w:end_w]
            out[row, col] = np.mean(block, dtype=np.float64)

    return out

def main():
    # Connect to Redis
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)

    # Open the camera
    cam = open_camera()
    if not cam:
        return

    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height} (expected 240x180)")

    print("camera_data.py is now streaming depth data to Redis...")

    try:
        while True:
            frame = cam.requestFrame(2000)  # Wait up to 2 seconds
            if frame is not None and isinstance(frame, ac.DepthData):
                # Convert to float64 right away
                depth_buf = frame.depth_data.astype(np.float64, copy=False)
                confidence_buf = frame.confidence_data.astype(np.float64, copy=False)

                # 1) Remove low-confidence pixels -> set them to 0
                depth_buf = np.where(confidence_buf < CONFIDENCE_THRESHOLD, 0, depth_buf)

                # 2) Clamp all values above 600 to 600
                depth_buf = np.clip(depth_buf, 0, MAX_DISTANCE)

                # 3) Convert to 12Ã—9 array by grouped averaging (float64)
                small_depth = block_average(depth_buf)

                # 4) Normalize each element to [0..1]
                small_depth /= MAX_DISTANCE

                # 5) Truncate each cell to 4 decimals in double precision
                small_depth = np.floor(small_depth * 10000) / 10000

                # Convert to list-of-lists for storing in Redis
                small_list = small_depth.tolist()

                rdb.set("camera_depth", repr(small_list))

                cam.releaseFrame(frame)

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping camera data script...")
    finally:
        cam.stop()
        cam.close()

if __name__ == "__main__":
    main()
