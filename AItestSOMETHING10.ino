#include <Wire.h>
 
#include <Adafruit_MPU6050.h>
 
#include <Adafruit_Sensor.h>
 
// Pin assignments for six ultrasonic sensors (Trig, Echo)
 
const int trigPins[6] = {13, 11, 9, 7, 5, 3};
 
const int echoPins[6] = {12, 10, 8, 6, 4, 2};
 
// Create two IMU objects
 
Adafruit_MPU6050 mpu1;
 
Adafruit_MPU6050 mpu2;
 
// Variables for heading calculation
 
float heading      = 0.0f;  // Integrated heading in degrees
 
float gzOffset     = 0.0f;  // Gyro Z offset in degrees/sec
 
unsigned long prevTime = 0; // For integration time step
 
// ---------------------------------------------------------------------------
 
// Basic function to measure distance (in cm) from one ultrasonic sensor
 
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
 
    // Set ultrasonic sensor pins
    for (int i = 0; i < 6; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
 
    // Initialize I2C and IMUs
    Wire.begin();
 
    if (!mpu1.begin(0x68)) {
        Serial.println("Could not find MPU6050 at 0x68.");
        while (1) { delay(10); }
    }
    if (!mpu2.begin(0x69)) {
        Serial.println("Could not find MPU6050 at 0x69.");
        while (1) { delay(10); }
    }
 
    // Configure both IMUs
    mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
 
    mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);
 
    // Calibrate Gz offset
    Serial.println("Calibrating gyro offset...");
    unsigned long startTime = millis();
    const unsigned long calibDuration = 2000;
 
    float sumGz = 0.0f;
    int count = 0;
 
    while (millis() - startTime < calibDuration) {
        sensors_event_t a1, g1, t1;
        sensors_event_t a2, g2, t2;
     
        mpu1.getEvent(&a1, &g1, &t1);
        mpu2.getEvent(&a2, &g2, &t2);
     
        float avgGz = (g1.gyro.z + g2.gyro.z) / 2.0f;
        sumGz += avgGz;
        count++;
        delay(5);
    }
 
    if (count > 0) {
        gzOffset = sumGz / (float)count;
    } else {
        gzOffset = 0.0f;
    }
 
    Serial.print("Gyro Z offset (deg/s): ");
    Serial.println(gzOffset, 4);
 
    // **Send "Calibrated" message**
    Serial.println("Calibrated");
 
    prevTime = millis();
}
 
 
// ---------------------------------------------------------------------------
 
void loop() {
 
  // Integrate heading continuously (whether or not 'P' is received)
 
  // so that heading is up to date whenever 'P' arrives
 
  unsigned long currentTime = millis();
 
  float dt = (currentTime - prevTime) / 1000.0f; // seconds
 
  prevTime = currentTime;
 
  // Read both IMUs
 
  sensors_event_t accel1, gyro1, temp1;
 
  sensors_event_t accel2, gyro2, temp2;
 
  mpu1.getEvent(&accel1, &gyro1, &temp1);
 
  mpu2.getEvent(&accel2, &gyro2, &temp2);
 
  // Average Gz
 
  float avgGz = (gyro1.gyro.z + gyro2.gyro.z) / 2.0f; // deg/s
 
  // Subtract offset and integrate
 
  float correctedGz = avgGz - gzOffset; // deg/s
 
  heading += correctedGz * dt;          // deg
 
  // Check for 'P' command to print distance and heading
 
  if (Serial.available() > 0) {
 
    char incoming = Serial.read();
 
    if (incoming == 'P') {
 
      // Read distances from all six ultrasonic sensors
 
      float distances[6];
 
      for (int i = 0; i < 6; i++) {
 
        distances[i] = getDistanceCM(trigPins[i], echoPins[i]);
 
      }
 
      // Print data in comma-separated format:
 
      // 1) Ultrasonic distances
 
      for (int i = 0; i < 6; i++) {
 
        Serial.print(distances[i], 2);
 
        Serial.print(",");
 
      }
 
      // 2) Current heading
 
      Serial.print(heading, 2);
 
      Serial.println(); // End line
 
    }
 
  }
 
}
 
 
 
