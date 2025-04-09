# 📦 Install packages
!pip install torch torchvision pandas matplotlib scikit-learn

# 📚 Imports
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from google.colab import files
import io
from collections import Counter

# 📁 Upload CSV
uploaded = files.upload()
dfs = [pd.read_csv(io.BytesIO(uploaded[f]), encoding='ISO-8859-1') for f in uploaded]
df = pd.concat(dfs, ignore_index=True)
print("Dataset shape:", df.shape)
print(df.head())

# 📊 Preprocess
df["Heading_sin"] = np.sin(2 * np.pi * df["Heading (0-1)"])
df["Heading_cos"] = np.cos(2 * np.pi * df["Heading (0-1)"])
sensor_cols = ["Heading_sin", "Heading_cos", "Dist1 (cm)", "Dist2 (cm)", "Dist3 (cm)", "Dist4 (cm)"]
camera_cols = [f"Depth_{i}_{j} (0-1)" for i in range(9) for j in range(12)]
motor_cols = ["Motor_IN1", "Motor_IN2", "Motor_IN3", "Motor_IN4"]

# Normalize
X_sensor_all = df[sensor_cols].to_numpy(dtype=np.float32)
X_sensor_all[:, 0:2] *= 2.0  # Heading x2
# Leave ultrasonic sensors at x1
X_cam_all = df[camera_cols].to_numpy(dtype=np.float32).reshape(-1, 1, 9, 12)
y_all = df[motor_cols].to_numpy(dtype=np.int32)

# Convert to rolling sequences
SEQLEN = 10
X_sensor_seq, X_cam_seq, y_seq = [], [], []

valid_actions = [
    [1, 0, 1, 0],  # forward
    [0, 1, 0, 1],  # reverse
    [1, 0, 0, 1],  # left
    [0, 1, 1, 0]   # right
]
action_labels = {tuple(a): i for i, a in enumerate(valid_actions)}

for i in range(SEQLEN, len(df)):
    label = tuple(y_all[i])
    if label in action_labels:
        X_sensor_seq.append(X_sensor_all[i-SEQLEN:i])
        X_cam_seq.append(X_cam_all[i-SEQLEN:i])
        y_seq.append(action_labels[label])

X_sensor_seq = np.stack(X_sensor_seq)
X_cam_seq = np.stack(X_cam_seq)
y_seq = np.array(y_seq)

print("After sequence processing:", X_sensor_seq.shape, X_cam_seq.shape, y_seq.shape)
print("Class distribution:", Counter(y_seq))

# 🔀 Train/test split
X_cam_train, X_cam_test, X_sensor_train, X_sensor_test, y_train, y_test = train_test_split(
    X_cam_seq, X_sensor_seq, y_seq, test_size=0.2, random_state=42
)

# 📦 Dataset
class TemporalDataset(torch.utils.data.Dataset):
    def __init__(self, X_cam, X_sensor, y):
        self.X_cam = torch.tensor(X_cam, dtype=torch.float32)
        self.X_sensor = torch.tensor(X_sensor, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.long)

    def __len__(self):
        return len(self.X_cam)

    def __getitem__(self, idx):
        return self.X_cam[idx], self.X_sensor[idx], self.y[idx]

train_loader = torch.utils.data.DataLoader(
    TemporalDataset(X_cam_train, X_sensor_train, y_train),
    batch_size=32, shuffle=True)

test_loader = torch.utils.data.DataLoader(
    TemporalDataset(X_cam_test, X_sensor_test, y_test),
    batch_size=32)

# 🧠 Model
class ManualTemporalModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.sensor_fc = nn.Linear(6, 8)

        self.conv1 = nn.Conv2d(1, 8, 3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(8, 16, 3, padding=1)

        self.temporal_sensor_fc = nn.Linear(10 * 8, 32)
        self.temporal_cam_fc = nn.Linear(10 * (16 * 2 * 3), 64)

        self.fc1 = nn.Linear(32 + 64, 32)
        self.fc2 = nn.Linear(32, 16)
        self.out = nn.Linear(16, 4)

    def forward(self, cam_seq, sensor_seq):
        B, T, C, H, W = cam_seq.shape  # (B, 10, 1, 9, 12)
        cam_seq = cam_seq.view(B * T, C, H, W)
        x = F.relu(self.conv1(cam_seq))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(B, T, -1)
        cam_features = x.reshape(B, -1)

        sensor_seq = self.sensor_fc(sensor_seq)  # (B, T, 8)
        sensor_features = sensor_seq.reshape(B, -1)

        s_embed = F.relu(self.temporal_sensor_fc(sensor_features))
        c_embed = F.relu(self.temporal_cam_fc(cam_features))

        combined = torch.cat((s_embed, c_embed), dim=1)
        x = F.relu(self.fc1(combined))
        x = F.relu(self.fc2(x))
        return F.log_softmax(self.out(x), dim=1)

model = ManualTemporalModel()
criterion = nn.NLLLoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 🏋️‍♂️ Train
print("\n📈 Training...")
for epoch in range(30):
    model.train()
    total_loss, total_acc = 0, 0
    for x_cam, x_sensor, y_true in train_loader:
        optimizer.zero_grad()
        y_pred = model(x_cam, x_sensor)
        loss = criterion(y_pred, y_true)
        loss.backward()
        optimizer.step()
        acc = (y_pred.argmax(dim=1) == y_true).float().mean()
        total_loss += loss.item()
        total_acc += acc.item()
    print(f"Epoch {epoch+1:03} | Loss: {total_loss/len(train_loader):.4f} | Acc: {total_acc/len(train_loader)*100:.2f}%")

# 💾 Save model
torch.save(model.state_dict(), "model_temporal_manual.pt")
files.download("model_temporal_manual.pt")
print("📦 Temporal model saved and downloaded.")
