# =====================================
# GOOGLE COLAB - ROVER MODEL TRAINING
# =====================================

# Step 1: Install necessary dependencies
!pip install tensorflow pandas numpy matplotlib scikit-learn

# Step 2: Import required libraries
import tensorflow as tf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from google.colab import files
import io
import os
from sklearn.model_selection import train_test_split

# Step 3: Upload CSV files (one or more)
print("Please upload one or more CSV files (identical format).")
uploaded = files.upload()

# Combine multiple CSVs into one DataFrame
dfs = []
for fname in uploaded.keys():
    df_temp = pd.read_csv(io.BytesIO(uploaded[fname]))
    dfs.append(df_temp)
df = pd.concat(dfs, ignore_index=True)

print("\nAll CSVs have been combined into a single dataset.")
print("First few rows:")
print(df.head())

# Step 4: Preprocess Data (single-frame)
def preprocess_data(df):
    """
    Extracts single-frame features and labels from the dataset.
    We have 113 features: 1 heading, 4 distances, and 108 camera-depth values.
    """
    motor_cols = ["Motor_IN1", "Motor_IN2", "Motor_IN3", "Motor_IN4"]
    sensor_cols = ["Heading (0-1)", "Dist1 (cm)", "Dist2 (cm)", "Dist3 (cm)", "Dist4 (cm)"]
    for i in range(9):
        for j in range(12):
            sensor_cols.append(f"Depth_{i}_{j} (0-1)")

    X = df[sensor_cols].to_numpy(dtype=np.float32)  # shape: (num_samples, 113)
    y = df[motor_cols].to_numpy(dtype=np.int32)     # shape: (num_samples, 4)
    return X, y

X_single, y_single = preprocess_data(df)

# Step 4a: Create sequence dataset (10 frames from last 100)
def create_sequence_dataset(X, y, total_history=100, seq_length=10):
    """
    For each index i, pick 10 evenly spaced frames from the previous 100,
    flatten to (1130,) and pair with y[i].
    """
    X_seqs = []
    y_seqs = []
    for i in range(total_history, len(X)):
        indices = np.linspace(i - total_history, i - 1, seq_length, dtype=int)
        frames = X[indices]  # shape (10, 113)
        frames_flat = frames.flatten()  # shape (1130,)
        X_seqs.append(frames_flat)
        y_seqs.append(y[i])  # label from the current row
    return np.array(X_seqs, dtype=np.float32), np.array(y_seqs, dtype=np.int32)

X, y = create_sequence_dataset(X_single, y_single,
                               total_history=100,
                               seq_length=10)

# Split into train/test sets
X_train, X_test, y_train, y_test = train_test_split(X, y,
                                                    test_size=0.2,
                                                    random_state=42)

# Step 5: Build Neural Network Model
model = tf.keras.Sequential([
    tf.keras.layers.Dense(128, activation='relu', input_shape=(X.shape[1],)),
    tf.keras.layers.Dense(64, activation='relu'),
    tf.keras.layers.Dense(32, activation='relu'),
    tf.keras.layers.Dense(4, activation='sigmoid')  # 4 output neurons for motor bits
])

model.compile(optimizer='adam',
              loss='binary_crossentropy',
              metrics=['accuracy'])

# Step 6: Train Model
print("\nTraining the model...")
history = model.fit(X_train, y_train, epochs=50, batch_size=32,
                    validation_data=(X_test, y_test), verbose=1)

# Step 7: Evaluate Model
test_loss, test_acc = model.evaluate(X_test, y_test, verbose=2)
print(f"\nTest Accuracy: {test_acc * 100:.2f}%")

# Step 8: Convert Model to TensorFlow Lite
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

tflite_filename = "model_ai_camera_stacked.tflite"
with open(tflite_filename, "wb") as f:
    f.write(tflite_model)

print(f"\nTFLite model saved as {tflite_filename}")

# Step 9: Download the TFLite model
files.download(tflite_filename)
