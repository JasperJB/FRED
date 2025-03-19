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
print("Please upload your CSV file (time-ordered):")
uploaded = files.upload()
filename = list(uploaded.keys())[0]
print("Uploaded file:", filename)

# ================================
# Colab Cell [3] - Read & Parse
# ================================
df = pd.read_csv(io.StringIO(uploaded[filename].decode('utf-8')))

def parse_motor_bits(s):
    return list(map(int, s.split(",")))

def parse_camera_data(camera_str):
    arr_2d = ast.literal_eval(camera_str)
    flat = []
    for row in arr_2d:
        flat.extend(row)
    return flat

motor_bits_series = df["Motor State"].apply(parse_motor_bits)
motor_bits = np.array(motor_bits_series.to_list(), dtype=np.float32)  # shape: (samples,4)

camera_series = df["Camera Depth 9x12"].apply(parse_camera_data)
camera_data = np.array(camera_series.to_list(), dtype=np.float32)  # shape: (samples,108)

heading = df["Heading (0-1)"].values.astype(np.float32).reshape(-1, 1)  # shape: (samples,1)

X_single = np.hstack([heading, camera_data])  # shape: (samples,109)
y_single = motor_bits

# ================================
# Colab Cell [4] - Create Stacked Inputs
# ================================
HISTORY_FRAMES = 3  # Number of past frames to stack

def create_stacked_inputs(X, y, history=HISTORY_FRAMES):
    """
    Creates an input dataset where each row contains 'history' frames of past data.
    """
    X_seq, y_seq = [], []
    for i in range(len(X) - history + 1):
        stacked = X[i : i + history].flatten()  # shape (history*109,)
        X_seq.append(stacked)
        y_seq.append(y[i + history - 1])  # Corresponding label
    return np.array(X_seq), np.array(y_seq)

X, y = create_stacked_inputs(X_single, y_single, HISTORY_FRAMES)

print("X shape:", X.shape)  # (samples, history*109)
print("y shape:", y.shape)  # (samples, 4)
print("Example X[0]:", X[0])
print("Example y[0]:", y[0])

# ================================
# Colab Cell [5] - Build & Train Model
# ================================
model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(HISTORY_FRAMES * 109,)),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(4, activation='sigmoid'),
])

model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

EPOCHS = 50
BATCH_SIZE = 32
history = model.fit(X, y, epochs=EPOCHS, batch_size=BATCH_SIZE, validation_split=0.1, verbose=1)

# ================================
# Colab Cell [6] - Convert to TFLite
# ================================
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

with open("model_ai_camera_stacked.tflite", "wb") as f:
    f.write(tflite_model)

files.download("model_ai_camera_stacked.tflite")
print("Download initiated. Done!")
