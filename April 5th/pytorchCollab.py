# 📦 Install packages
!pip install torch torchvision pandas matplotlib scikit-learn

# 📚 Imports
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torch.nn.utils.prune as prune
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from google.colab import files
import io

# 📁 Upload CSV
uploaded = files.upload()
dfs = [pd.read_csv(io.BytesIO(uploaded[f]), encoding='ISO-8859-1') for f in uploaded]
df = pd.concat(dfs, ignore_index=True)
print("Dataset shape:", df.shape)
print(df.head())

# 📊 Preprocess
sensor_cols = ["Heading (0-1)", "Dist1 (cm)", "Dist2 (cm)", "Dist3 (cm)", "Dist4 (cm)"]
camera_cols = [f"Depth_{i}_{j} (0-1)" for i in range(9) for j in range(12)]
motor_cols = ["Motor_IN1", "Motor_IN2", "Motor_IN3", "Motor_IN4"]

X_sensor = df[sensor_cols].to_numpy(dtype=np.float32)
X_cam = df[camera_cols].to_numpy(dtype=np.float32).reshape(-1, 1, 9, 12)
y = df[motor_cols].to_numpy(dtype=np.float32)

# 🎯 Keep only 4 valid actions
valid_actions = [
    [1, 0, 1, 0],  # forward
    [0, 1, 0, 1],  # reverse
    [1, 0, 0, 1],  # left
    [0, 1, 1, 0]   # right
]
mask = np.any([np.all(y == a, axis=1) for a in valid_actions], axis=0)
X_sensor = X_sensor[mask]
X_cam = X_cam[mask]
y = y[mask]

# ✂️ Show class distribution
unique, counts = np.unique(y, axis=0, return_counts=True)
print("Class distribution:")
for u, c in zip(unique, counts):
    print(f"{u} → {c} samples")

# 🔀 Train/test split
X_cam_train, X_cam_test, X_sensor_train, X_sensor_test, y_train, y_test = train_test_split(
    X_cam, X_sensor, y, test_size=0.2, random_state=42
)

# 📦 Dataset
class DualInputDataset(torch.utils.data.Dataset):
    def __init__(self, X_cam, X_sensor, y):
        self.X_cam = torch.tensor(X_cam, dtype=torch.float32)
        self.X_sensor = torch.tensor(X_sensor, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.float32)

    def __len__(self):
        return len(self.X_cam)

    def __getitem__(self, idx):
        return self.X_cam[idx], self.X_sensor[idx], self.y[idx]

train_loader = torch.utils.data.DataLoader(
    DualInputDataset(X_cam_train, X_sensor_train, y_train),
    batch_size=32, shuffle=True)

test_loader = torch.utils.data.DataLoader(
    DualInputDataset(X_cam_test, X_sensor_test, y_test),
    batch_size=32)

# 🧠 Model
class LightweightDualBranchCNN(nn.Module):
    def __init__(self):
        super(LightweightDualBranchCNN, self).__init__()
        self.sensor_fc = nn.Linear(5, 8)
        self.conv1 = nn.Conv2d(1, 8, kernel_size=3, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(8, 16, kernel_size=3, padding=1)
        self.fc1 = nn.Linear(16*2*3 + 8, 32)
        self.fc2 = nn.Linear(32, 16)
        self.out = nn.Linear(16, 4)

    def forward(self, sensor_input, cam_input):
        sensor = F.relu(self.sensor_fc(sensor_input))
        x = F.relu(self.conv1(cam_input))
        x = self.pool(x)
        x = F.relu(self.conv2(x))
        x = self.pool(x)
        x = x.view(x.size(0), -1)
        x = torch.cat((x, sensor), dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = torch.sigmoid(self.out(x))
        return x

model = LightweightDualBranchCNN()
criterion = nn.BCELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 🏋️‍♂️ Train
print("\n📈 Training...")
for epoch in range(15):
    model.train()
    total_loss, total_acc = 0, 0
    for x_cam, x_sensor, y_true in train_loader:
        optimizer.zero_grad()
        y_pred = model(x_sensor, x_cam)
        loss = criterion(y_pred, y_true)
        loss.backward()
        optimizer.step()
        acc = (y_pred.round() == y_true).all(dim=1).float().mean()
        total_loss += loss.item()
        total_acc += acc.item()
    print(f"Epoch {epoch+1:03} | Loss: {total_loss/len(train_loader):.4f} | Acc: {total_acc/len(train_loader)*100:.2f}%")

# ✂️ Prune linear layers (40% weights)
for layer in [model.sensor_fc, model.fc1, model.fc2, model.out]:
    prune.l1_unstructured(layer, name="weight", amount=0.4)
    prune.remove(layer, "weight")

# 💾 Save model
torch.save(model.state_dict(), "model_ai_camera_stacked_pruned.pt")
files.download("model_ai_camera_stacked_pruned.pt")
print("📦 Pruned model saved and downloaded.") 
