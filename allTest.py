from rich.progress import Progress
import serial
import time
import numpy as np
from pykalman import KalmanFilter

# Serial port settings
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
CALIBRATION_SAMPLES = 20  # Number of samples to collect for calibration

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
    exit()

# ---------------- CALIBRATION ----------------

def collect_calibration_data():
    """Collects IMU samples for offset correction."""
    print("\n[INFO] Collecting calibration data...")
    imu_data = []

    # Use a rich progress bar
    with Progress() as progress:
        task = progress.add_task("[cyan]Calibrating...", total=CALIBRATION_SAMPLES)
        
        for _ in range(CALIBRATION_SAMPLES):
            line = ser.readline().decode("utf-8").strip()
            if line.startswith("IMU:"):
                try:
                    imu_values = list(map(float, line.split(";")[0][4:].split(",")))
                    imu_data.append(imu_values)
                except ValueError:
                    continue  # Ignore faulty readings

            progress.advance(task, 1)
            time.sleep(0.05)  # Small delay to allow sensor readings

    imu_array = np.array(imu_data)
    
    # Compute offsets (average of collected values)
    imu_offsets = np.mean(imu_array, axis=0)

    # Compute standard deviation (for noise estimation)
    imu_std_dev = np.std(imu_array, axis=0)

    print("\n[INFO] Calibration Complete!")
    print(f"IMU Offsets: {imu_offsets}")
    print(f"IMU Standard Deviations: {imu_std_dev}")

    return imu_offsets

# ---------------- FILTERING ----------------

def initialize_kalman_filter():
    """Initializes a Kalman filter for IMU data."""
    transition_matrices = np.eye(6)  # Identity matrix (each axis independent)
    observation_matrices = np.eye(6)

    kf = KalmanFilter(
        transition_matrices=transition_matrices,
        observation_matrices=observation_matrices,
        initial_state_mean=np.zeros(6),
        observation_covariance=np.eye(6) * 0.1,  # Estimated measurement noise
        transition_covariance=np.eye(6) * 0.01   # Process noise
    )
    return kf

def apply_kalman_filter(kf, data, prev_state, prev_covariance):
    """Applies Kalman filtering to smooth out noise."""
    state_mean, state_covariance = kf.filter_update(
        prev_state,
        prev_covariance,
        observation=data
    )
    return state_mean, state_covariance

# ---------------- MAIN PROCESS ----------------

def process_imu_and_sonar_data():
    """Reads, calibrates, and processes IMU and SONAR data from the Arduino."""
    imu_offsets = collect_calibration_data()
    
    kf = initialize_kalman_filter()
    prev_state = np.zeros(6)  # Initial state mean
    prev_covariance = np.eye(6) * 0.1  # Initial state covariance

    print("\n[INFO] Now reading and processing IMU and SONAR data...\n")

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            
            if line.startswith("IMU:") and "SONAR:" in line:
                try:
                    # Parse IMU and SONAR data
                    raw_imu_data = list(map(float, line.split(";")[0][4:].split(",")))
                    raw_sonar_data = list(map(float, line.split(";")[1][7:].split(",")))

                    # Calibration: Remove IMU offsets
                    calibrated_imu_data = np.array(raw_imu_data) - imu_offsets

                    # Apply Kalman filtering to IMU data
                    filtered_imu_data, prev_covariance = apply_kalman_filter(
                        kf, calibrated_imu_data, prev_state, prev_covariance
                    )
                    prev_state = filtered_imu_data  # Update previous state

                    # Print filtered IMU and raw SONAR data
                    print(f"Filtered IMU: {filtered_imu_data.round(3)}")
                    print(f"Raw SONAR: {raw_sonar_data}")

                except ValueError:
                    continue  # Ignore faulty readings

        except KeyboardInterrupt:
            print("\n[INFO] Exiting gracefully...")
            break

    ser.close()

if __name__ == "__main__":
    process_imu_and_sonar_data()
