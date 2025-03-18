# -*- coding: utf-8 -*-
import redis
import time
import numpy as np
import tflite_runtime.interpreter as tflite
import ast

def load_tflite_model(tflite_path):
    interpreter = tflite.Interpreter(model_path=tflite_path)
    interpreter.allocate_tensors()
    return interpreter

def get_heading(r):
    val = r.get("arduino_heading")
    if val is None:
        return None
    try:
        return float(val)
    except ValueError:
        return None

def get_camera_data(r):
    #Fetches the 9×12 normalized depth array from Redis (key: "camera_depth"),
    #which is stored as a repr(...) of a list-of-lists.
    #Flattens it to shape (108,).
    val = r.get("camera_depth")
    if val is None:
        return None
    try:
        arr_2d = ast.literal_eval(val.decode())  # e.g. [[0.0, 0.001, ...], [...], ...]
        flat = []
        for row in arr_2d:
            flat.extend(row)  # add each row's 12 columns
        return np.array(flat, dtype=np.float32)  # shape (108,)
    except:
        return None

def run_inference(interpreter, heading, camera_flat):
    """
    The model expects shape: (1, 109):
      - 1 float for heading
      - 108 floats for camera
    """
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Combine heading (1,) + camera_flat (108,) => total 109
    input_arr = np.concatenate(([heading], camera_flat), axis=0)  # shape (109,)
    input_arr = input_arr.reshape(1, -1).astype(np.float32)       # shape (1,109)

    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], input_arr)
    
    # Run inference
    interpreter.invoke()
    
    # Retrieve output: shape (1,4)
    output_data = interpreter.get_tensor(output_details[0]['index'])
    motor_bits = output_data[0]  # shape (4,)
    
    # Convert each float to a bit (threshold=0.5)
    motor_bits_thresholded = (motor_bits >= 0.5).astype(int)
    return motor_bits_thresholded

def main():
    # 1) Connect to Redis
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    
    # 2) Load TFLite model that expects 109 inputs
    tflite_path = "ai_model.tflite"  # The path to your .tflite file
    interpreter = load_tflite_model(tflite_path)
    
    print("ai_inference.py: TFLite model (heading+camera) loaded. Beginning inference loop...")
    
    try:
        while True:
            heading = get_heading(r)
            camera_flat = get_camera_data(r)

            # Only run inference if both heading + camera are valid
            if heading is not None and camera_flat is not None:
                # shape (4,)
                motor_bits = run_inference(interpreter, heading, camera_flat)
                
                # Publish to motor_command
                cmd_str = ",".join(str(bit) for bit in motor_bits)
                r.publish("motor_command", cmd_str)
                r.set("motor_state", cmd_str)
                
                print(f"\rInference - heading={heading:.3f} => {cmd_str}", end="", flush=True)
            
            # Sleep a bit to avoid hammering CPU
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping AI inference script.")

if __name__ == "__main__":
    main()
