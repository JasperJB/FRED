# inference_stacked_ai.py
# -*- coding: utf-8 -*-

import redis
import time
import numpy as np
import ast
import tflite_runtime.interpreter as tflite
from collections import deque

#####################
# CONFIG
#####################
HISTORY_FRAMES = 3  # Must match training setting
TFLITE_PATH = "model_ai_camera_stacked.tflite"

def load_tflite_model(path):
    interpreter = tflite.Interpreter(model_path=path)
    interpreter.allocate_tensors()
    return interpreter

def get_heading(rdb):
    val = rdb.get("arduino_heading")
    if val is None:
        return None
    try:
        return float(val.decode())
    except:
        return None

def get_camera_data(rdb):
    val = rdb.get("camera_depth")
    if val is None:
        return None
    try:
        arr_2d = ast.literal_eval(val.decode())  # 9x12
        flat = []
        for row in arr_2d:
            flat.extend(row)
        return np.array(flat, dtype=np.float32)  # shape (108,)
    except:
        return None

def main():
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)

    # Load TFLite model
    interpreter = load_tflite_model(TFLITE_PATH)

    # Gather input/output tensor details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Store the last HISTORY_FRAMES frames
    frame_buffer = deque(maxlen=HISTORY_FRAMES)

    print(f"Stacked AI Inference: expecting (history={HISTORY_FRAMES}, 109).")

    try:
        while True:
            heading = get_heading(rdb)
            camera_flat = get_camera_data(rdb)

            if heading is None or camera_flat is None:
                time.sleep(0.1)
                continue

            # Combine (heading + camera) => shape (109,)
            current_frame = np.concatenate(([heading], camera_flat), axis=0)
            frame_buffer.append(current_frame)

            # Only run inference once we have enough history
            if len(frame_buffer) == HISTORY_FRAMES:
                # shape => (1, HISTORY_FRAMES * 109)
                input_data = np.array(frame_buffer, dtype=np.float32).flatten().reshape(1, -1)

                # TFLite Inference
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                output_data = interpreter.get_tensor(output_details[0]['index'])[0]  # shape: (4,)

                motor_bits = (output_data >= 0.5).astype(int)
                cmd_str = ",".join(str(b) for b in motor_bits)

                # Publish to 'motor_command'
                rdb.publish("motor_command", cmd_str)
                rdb.set("motor_state", cmd_str)

                print(f"\rStacked Inference => {cmd_str}", end="", flush=True)

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping stacked AI inference script.")

if __name__ == "__main__":
    main()
