#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pin assignments for six ultrasonic sensors (Trig, Echo):
const int trigPins[6] = {13, 11,  9,  7,  5,  3};
const int echoPins[6] = {12, 10,  8,  6,  4,  2};

// Create two IMU objects:
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

// ---------------------------------------------------------------------------
// Basic function to measure distance (in cm) from one ultrasonic sensor:
float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343f / 2.0f; 
  return distance;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // Set ultrasonic pins:
  for (int i = 0; i < 6; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Initialize I2C and IMUs:
  Wire.begin();

  if (!mpu1.begin(0x68)) {
    Serial.println("Could not find MPU6050 at 0x68.");
    while (1) { delay(10); }
  }
  if (!mpu2.begin(0x69)) {
    Serial.println("Could not find MPU6050 at 0x69.");
    while (1) { delay(10); }
  }

  // Example configurations for both IMUs:
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// ---------------------------------------------------------------------------
void loop() {
  // Check if 'P' is received:
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 'P') {
      // Read distances from all six ultrasonic sensors:
      float distances[6];
      for (int i = 0; i < 6; i++) {
        distances[i] = getDistanceCM(trigPins[i], echoPins[i]);
      }

      // Read from both IMUs:
      sensors_event_t accel1, gyro1, temp1;
      sensors_event_t accel2, gyro2, temp2;
      mpu1.getEvent(&accel1, &gyro1, &temp1);
      mpu2.getEvent(&accel2, &gyro2, &temp2);

      // Average the IMU readings:
      float avgAx = (accel1.acceleration.x + accel2.acceleration.x) / 2.0f;
      float avgAy = (accel1.acceleration.y + accel2.acceleration.y) / 2.0f;
      float avgAz = (accel1.acceleration.z + accel2.acceleration.z) / 2.0f;
      float avgGx = (gyro1.gyro.x + gyro2.gyro.x) / 2.0f;
      float avgGy = (gyro1.gyro.y + gyro2.gyro.y) / 2.0f;
      float avgGz = (gyro1.gyro.z + gyro2.gyro.z) / 2.0f;

      // Print data in comma-separated format:
      // 1) Ultrasonic distances:
      for (int i = 0; i < 6; i++) {
        Serial.print(distances[i], 2);
        Serial.print(",");
      }
      // 2) Averaged IMU data: accel (x, y, z), gyro (x, y, z)
      Serial.print(avgAx, 2); Serial.print(",");
      Serial.print(avgAy, 2); Serial.print(",");
      Serial.print(avgAz, 2); Serial.print(",");
      Serial.print(avgGx, 2); Serial.print(",");
      Serial.print(avgGy, 2); Serial.print(",");
      Serial.print(avgGz, 2);

      Serial.println(); // End line
    }
  }
}
