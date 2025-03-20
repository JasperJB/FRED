# inference_ai.py
# -*- coding: utf-8 -*-
import redis
import time
import numpy as np
import ast
import tflite_runtime.interpreter as tflite
from collections import deque

HISTORY_FRAMES = 10
FEATURES_PER_FRAME = 113  # Heading (1) + 4 distances + 108-depth values

def get_redis_float(rdb, key):
    val = rdb.get(key)
    if val is None:
        return None
    try:
        return float(val.decode())
    except:
        return None

def get_redis_list(rdb, key):
    """Retrieve a list (or list of lists) stored as a string in Redis."""
    val = rdb.get(key)
    if val is None:
        return None
    try:
        return ast.literal_eval(val.decode())
    except:
        return None

def main():
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)
    interpreter = tflite.Interpreter(model_path="model_ai_camera_stacked.tflite")
    interpreter.allocate_tensors()

    frame_buffer = deque(maxlen=HISTORY_FRAMES)

    try:
        while True:
            # Get all sensor inputs from Redis
            heading = get_redis_float(rdb, "arduino_heading")
            dist1 = get_redis_float(rdb, "arduino_dist1")
            dist2 = get_redis_float(rdb, "arduino_dist2")
            dist3 = get_redis_float(rdb, "arduino_dist3")
            dist4 = get_redis_float(rdb, "arduino_dist4")
            camera_data = get_redis_list(rdb, "camera_depth")

            if None in (heading, dist1, dist2, dist3, dist4) or camera_data is None:
                time.sleep(0.1)
                continue

            # Flatten the 9x12 camera depth matrix into a list of 108 values
            depth_flat = [val for row in camera_data for val in row]
            current_frame = np.array(
                [heading, dist1, dist2, dist3, dist4] + depth_flat, dtype=np.float32
            )
            frame_buffer.append(current_frame)

            if len(frame_buffer) == HISTORY_FRAMES:
                # Prepare input tensor with shape (1, HISTORY_FRAMES * FEATURES_PER_FRAME)
                input_data = np.array(frame_buffer, dtype=np.float32).flatten().reshape(1, -1)
                interpreter.set_tensor(interpreter.get_input_details()[0]['index'], input_data)
                interpreter.invoke()
                # Interpret model output (e.g., sigmoid outputs) as binary motor state
                motor_bits = (interpreter.get_tensor(interpreter.get_output_details()[0]['index'])[0] >= 0.5).astype(int)
                # Publish motor command and update motor state in Redis
                cmd_str = ",".join(map(str, motor_bits))
                rdb.publish("motor_command", cmd_str)
                rdb.set("motor_state", cmd_str)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping AI inference.")

if __name__ == "__main__":
    main()
