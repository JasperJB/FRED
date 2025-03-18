# ================================
# Colab Cell [1] - Install & Imports
# ================================
!pip install tensorflow==2.12

import tensorflow as tf
import pandas as pd
import numpy as np
import io
import ast
from google.colab import files

print("Setup complete. TensorFlow version:", tf.__version__)

# ================================
# Colab Cell [2] - Upload CSV
# ================================
print("Please upload your CSV file (with headings + camera data + motor state):")
uploaded = files.upload()

filename = list(uploaded.keys())[0]
print("Uploaded file:", filename)

# ================================
# Colab Cell [3] - Read & Parse CSV
# ================================
# We assume your CSV has columns:
#   1) "Timestamp"
#   2) "Heading (0-1)"
#   3) "Motor State" (e.g. "1,0,1,0")
#   4) "Camera Depth 9x12" (string of a 9x12 list-of-lists)

df = pd.read_csv(io.StringIO(uploaded[filename].decode('utf-8')))

# -- Parse "Motor State" into [0,1,0,1] etc. --
def parse_motor_bits(state_str):
    return list(map(int, state_str.split(",")))

motor_bits_series = df["Motor State"].apply(parse_motor_bits)
motor_bits = np.array(motor_bits_series.to_list(), dtype=np.float32)  # shape: (samples,4)

# -- Parse heading --
heading = df["Heading (0-1)"].values.astype(np.float32).reshape(-1, 1)  # shape: (samples,1)

# -- Parse camera data (9x12) into a flattened 108-length array --
def parse_camera_data(camera_str):
    # camera_str is something like "[[0.0000, 0.0001, ...], [ ... ], ... ]"
    arr_2d = ast.literal_eval(camera_str)  # 9x12 list of lists
    flat = []
    for row in arr_2d:
        flat.extend(row)  # add the 12 columns
    return flat

camera_series = df["Camera Depth 9x12"].apply(parse_camera_data)
camera_data = np.array(camera_series.to_list(), dtype=np.float32)  # shape: (samples,108)

# -- Combine heading (1) + camera_data (108) => 109 inputs total --
X = np.hstack([heading, camera_data])  # shape: (samples,109)
y = motor_bits  # shape: (samples,4)

print("Data shapes:")
print(" X:", X.shape, "y:", y.shape)
print("Example X[0]:", X[0])
print("Example y[0]:", y[0])

# ================================
# Colab Cell [4] - Build & Train Model
# ================================
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(109,)),    # 1 heading + 108 camera floats
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(32, activation='relu'),
    tf.keras.layers.Dense(4, activation='sigmoid')  # 4 motor bits in [0..1]
])

model.compile(
    loss='binary_crossentropy',
    optimizer='adam',
    metrics=['accuracy']
)

EPOCHS = 20
BATCH_SIZE = 32

history = model.fit(
    X, y,
    epochs=EPOCHS,
    batch_size=BATCH_SIZE,
    validation_split=0.1,
    verbose=1
)

# ================================
# Colab Cell [5] - Convert to TFLite
# ================================
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

# ================================
# Colab Cell [6] - Download TFLite
# ================================
with open("model_ai_camera.tflite", "wb") as f:
    f.write(tflite_model)

print("TFLite file created: model_ai_camera.tflite")

files.download("model_ai_camera.tflite")
print("Download initiated. Training & conversion complete!")
