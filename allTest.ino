#include <Wire.h>

// Define IMU I2C addresses
#define IMU1_ADDRESS 0x68 // AD0 connected to GND
#define IMU2_ADDRESS 0x69 // AD0 connected to 3.3V

// Sonar sensor pins
#define TRIG_PIN 5
const int ECHO_PINS[6] = {2, 3, 4, 6, 7, 8};

// Variables to store IMU data
int16_t ax1, ay1, az1, gx1, gy1, gz1;
int16_t ax2, ay2, az2, gx2, gy2, gz2;
float avg_ax_mps2, avg_ay_mps2, avg_az_mps2, avg_gx_dps, avg_gy_dps, avg_gz_dps;
uint16_t sonar_distances[6] = {0, 0, 0, 0, 0, 0};

// Sensitivity constants
const float ACCEL_SENSITIVITY = 16384.0; // ±2g
const float GYRO_SENSITIVITY = 131.0;    // ±250 dps
const float GRAVITY = 9.8;               // m/s²

// Debugging flag
#define DEBUG 1

void setup() {
    Serial.begin(115200);

    // Initialize I2C for IMUs
    Wire1.begin(A3, A0); // SDA = A3, SCL = A0
    Wire1.setClock(100000);

    // Initialize sonar pins
    pinMode(TRIG_PIN, OUTPUT);
    for (int i = 0; i < 6; i++) pinMode(ECHO_PINS[i], INPUT);

    // Initialize IMUs
    initializeIMU(IMU1_ADDRESS);
    initializeIMU(IMU2_ADDRESS);

    if (DEBUG) Serial.println("IMUs initialized.");
}

void loop() {
    // Read data from IMUs
    readIMUData(IMU1_ADDRESS, ax1, ay1, az1, gx1, gy1, gz1);
    readIMUData(IMU2_ADDRESS, ax2, ay2, az2, gx2, gy2, gz2);

    // Average IMU data
    averageIMUData();

    // Read sonar data
    readSonarData();

    // Send data over Serial
    sendDataSerial();

    delay(50);
}

void initializeIMU(uint8_t imu_address) {
    Wire1.beginTransmission(imu_address);
    Wire1.write(0x6B); // Power management register
    Wire1.write(0x00); // Wake up the IMU
    Wire1.endTransmission();
}

void readIMUData(uint8_t imu_address, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire1.beginTransmission(imu_address);
    Wire1.write(0x3B); // Starting register for accelerometer data
    Wire1.endTransmission(false);
    Wire1.requestFrom(imu_address, 14);

    ax = (Wire1.read() << 8) | Wire1.read();
    ay = (Wire1.read() << 8) | Wire1.read();
    az = (Wire1.read() << 8) | Wire1.read();
    Wire1.read(); Wire1.read(); // Skip temperature data
    gx = (Wire1.read() << 8) | Wire1.read();
    gy = (Wire1.read() << 8) | Wire1.read();
    gz = (Wire1.read() << 8) | Wire1.read();
}

void averageIMUData() {
    avg_ax_mps2 = (((ax1 + ax2) / 2.0) / ACCEL_SENSITIVITY) * GRAVITY;
    avg_ay_mps2 = (((ay1 + ay2) / 2.0) / ACCEL_SENSITIVITY) * GRAVITY;
    avg_az_mps2 = (((az1 + az2) / 2.0) / ACCEL_SENSITIVITY) * GRAVITY;
    avg_gx_dps = ((gx1 + gx2) / 2.0) / GYRO_SENSITIVITY;
    avg_gy_dps = ((gy1 + gy2) / 2.0) / GYRO_SENSITIVITY;
    avg_gz_dps = ((gz1 + gz2) / 2.0) / GYRO_SENSITIVITY;
}

void readSonarData() {
    for (int i = 0; i < 6; i++) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        long duration = pulseIn(ECHO_PINS[i], HIGH, 30000);
        sonar_distances[i] = duration == 0 ? 0 : duration * 0.034 / 2;

        delay(100);
    }
}

void sendDataSerial() {
    Serial.print("IMU:");
    Serial.print(avg_ax_mps2); Serial.print(",");
    Serial.print(avg_ay_mps2); Serial.print(",");
    Serial.print(avg_az_mps2); Serial.print(",");
    Serial.print(avg_gx_dps); Serial.print(",");
    Serial.print(avg_gy_dps); Serial.print(",");
    Serial.print(avg_gz_dps);

    Serial.print("; SONAR:");
    for (int i = 0; i < 6; i++) {
        Serial.print(sonar_distances[i]);
        if (i < 5) Serial.print(",");
    }
    Serial.println();
}
