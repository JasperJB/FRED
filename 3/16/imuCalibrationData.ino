#include <Wire.h>

// --- I2C addresses for the two MPU6050s ---
#define MPU1_ADDRESS 0x68
#define MPU2_ADDRESS 0x69

// --- Calibration settings ---
#define CALIBRATION_SAMPLES 100
float mpu1_gzOffset = 0.0;
float mpu2_gzOffset = 0.0;

// --- Kalman Filter Variables (for 1D on gyro Z) ---
float x_est = 0.0;      // Filtered value (estimate) of gyro Z
float p_est = 1.0;      // Estimate error covariance
float Q = 0.01;         // Process noise covariance (tune as needed)
float R = 5.0;          // Measurement noise covariance (tune as needed)

// --- Heading ---
float heading = 0.0;
unsigned long lastTime = 0;

// --- Structure to hold raw IMU data ---
struct SensorData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

// -------------------------------------------------------------
// Initialize a single MPU6050 by waking it from sleep
void initMPU6050(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Set to zero (wake up MPU6050)
  Wire.endTransmission(true);
}

// -------------------------------------------------------------
// Read raw accelerometer & gyroscope data from a given MPU6050
SensorData readMPU6050(uint8_t address) {
  SensorData data;
  
  Wire.beginTransmission(address);
  Wire.write(0x3B);               // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)14, (uint8_t)true);

  data.ax = (Wire.read() << 8) | Wire.read();  // AX
  data.ay = (Wire.read() << 8) | Wire.read();  // AY
  data.az = (Wire.read() << 8) | Wire.read();  // AZ
  
  // Skip temperature registers
  Wire.read(); 
  Wire.read();
  
  data.gx = (Wire.read() << 8) | Wire.read();  // GX
  data.gy = (Wire.read() << 8) | Wire.read();  // GY
  data.gz = (Wire.read() << 8) | Wire.read();  // GZ

  return data;
}

// -------------------------------------------------------------
// Calibrate each MPU by collecting multiple samples of GZ while stationary
// We find average offset for each device separately.
void calibrateMPU(uint8_t address, float &gzOffset) {
  float sum = 0.0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    SensorData d = readMPU6050(address);

    // Raw -> deg/s for ±250 deg/s => 131 LSB/(°/s)
    float gz = (float)d.gz / 131.0;
    sum += gz;

    delay(10); // short delay between samples
  }

  // Average offset
  gzOffset = sum / (float)CALIBRATION_SAMPLES;
}

// -------------------------------------------------------------
// Simple 1D Kalman filter update for a single measurement
//  measurement = raw gyro Z reading (deg/s), after offset correction
float kalmanUpdate(float measurement) {
  // Prediction step
  float x_pred = x_est;
  float p_pred = p_est + Q;

  // Measurement update
  float K = p_pred / (p_pred + R); // Kalman gain
  float x_new = x_pred + K * (measurement - x_pred);
  float p_new = (1.0f - K) * p_pred;

  // Update global estimates
  x_est = x_new;
  p_est = p_new;

  return x_est; // Return the filtered result
}

// -------------------------------------------------------------
void setup() {
  // For ESP32, often SDA=21, SCL=22. Change if needed.
  // If your board truly uses pins labeled A4 (SDA) and A5 (SCL), update accordingly:
  Wire.begin(21, 22);

  Serial.begin(115200);
  delay(2000); // give some time for Serial to start

  // Initialize both MPU6050s
  initMPU6050(MPU1_ADDRESS);
  initMPU6050(MPU2_ADDRESS);

  // --- Calibration ---
  Serial.println("Calibrating MPU1...");
  calibrateMPU(MPU1_ADDRESS, mpu1_gzOffset);
  Serial.print("MPU1 GZ Offset: ");
  Serial.println(mpu1_gzOffset);

  Serial.println("Calibrating MPU2...");
  calibrateMPU(MPU2_ADDRESS, mpu2_gzOffset);
  Serial.print("MPU2 GZ Offset: ");
  Serial.println(mpu2_gzOffset);

  lastTime = millis();
  Serial.println("Calibration complete.\n");
}

// -------------------------------------------------------------
void loop() {
  // Calculate time step
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Read both IMUs
  SensorData d1 = readMPU6050(MPU1_ADDRESS);
  SensorData d2 = readMPU6050(MPU2_ADDRESS);

  // Convert raw Z gyro to deg/s, then apply each sensor's offset
  float gyroZ1 = (float)d1.gz / 131.0 - mpu1_gzOffset;
  float gyroZ2 = (float)d2.gz / 131.0 - mpu2_gzOffset;

  // Average the corrected Z gyro
  float gyroZ_avg = (gyroZ1 + gyroZ2) * 0.5;

  // Kalman filter to smooth the averaged gyroZ
  float gyroZ_filtered = kalmanUpdate(gyroZ_avg);

  // Integrate the filtered Z rate to get heading
  heading += gyroZ_filtered * dt;

  // Keep heading in [0, 360) range
  if (heading < 0.0) {
    heading += 360.0;
  } else if (heading >= 360.0) {
    heading -= 360.0;
  }

  // Print to Serial Monitor
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print(" deg  |  Gyro Z (filtered): ");
  Serial.print(gyroZ_filtered);
  Serial.println(" deg/s");

  delay(10);
}
