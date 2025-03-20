#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

// Instantiate two MPU6050 objects with different I2C addresses
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

// Ultrasonic sensor pins
const int trig1 = 13, echo1 = 12;
const int trig2 = 9, echo2 = 8;
const int trig3 = 5, echo3 = 4;
const int trig4 = 3, echo4 = 2;

// Calibration offsets for the z-axis (gyro)
float offset1 = 0.0;
float offset2 = 0.0;

// Heading in degrees (integrated angular velocity)
float heading = 0.0;

// Timing variables
unsigned long lastPrintTime = 0;
unsigned long lastLoopTime = 0;

// Conversion factor for the default ±250°/s range (131 LSB per °/s)
const float GYRO_SCALE = 131.0;

// Rolling average buffers
const int ROLLING_WINDOW = 5;
float dist1Buffer[ROLLING_WINDOW] = {0};
float dist2Buffer[ROLLING_WINDOW] = {0};
float dist3Buffer[ROLLING_WINDOW] = {0};
float dist4Buffer[ROLLING_WINDOW] = {0};
int distIndex = 0;

void setup() {
  Wire.begin();            
  Serial.begin(115200);
  
  // Initialize both MPU6050 sensors
  mpu1.initialize();
  mpu2.initialize();
  
  // Set ultrasonic sensor pins
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
  
  delay(1000);
  
  // Allow sensors to stabilize
  delay(1000);
  
  // Calibrate by averaging 100 readings from each sensor's z-axis gyro
  const int numSamples = 100;
  long sum1 = 0;
  long sum2 = 0;

  for (int i = 0; i < numSamples; i++) {
    sum1 += mpu1.getRotationZ();
    sum2 += mpu2.getRotationZ();
    delay(10);
  }

  offset1 = (float)sum1 / numSamples;
  offset2 = (float)sum2 / numSamples;

  lastPrintTime = millis();
  lastLoopTime = micros();
}

void loop() {
  // Calculate time delta in seconds since last loop iteration
  unsigned long currentTime = micros();
  float dt = (currentTime - lastLoopTime) / 1000000.0;
  lastLoopTime = currentTime;

  // Obtain raw z-axis gyro readings and apply calibration offsets
  float gz1 = mpu1.getRotationZ() - offset1;
  float gz2 = mpu2.getRotationZ() - offset2;

  // Compute average gyro reading
  float avgGz = (gz1 + gz2) / 2.0;

  // Convert raw reading to degrees per second
  float rateDPS = avgGz / GYRO_SCALE;

  // Integrate angular velocity to update heading
  heading += rateDPS * dt;

  // Wrap heading into the range [0, 360)
  heading = fmod(heading, 360.0);
  if (heading < 0.0) {
    heading += 360.0;
  }

  // Read distances from ultrasonic sensors
  dist1Buffer[distIndex] = getDistance(trig1, echo1);
  dist2Buffer[distIndex] = getDistance(trig2, echo2);
  dist3Buffer[distIndex] = getDistance(trig3, echo3);
  dist4Buffer[distIndex] = getDistance(trig4, echo4);
  
  distIndex = (distIndex + 1) % ROLLING_WINDOW;

  // Compute rolling average for distance values
  float dist1Avg = 0, dist2Avg = 0, dist3Avg = 0, dist4Avg = 0;
  for (int i = 0; i < ROLLING_WINDOW; i++) {
    dist1Avg += dist1Buffer[i];
    dist2Avg += dist2Buffer[i];
    dist3Avg += dist3Buffer[i];
    dist4Avg += dist4Buffer[i];
  }
  dist1Avg /= ROLLING_WINDOW;
  dist2Avg /= ROLLING_WINDOW;
  dist3Avg /= ROLLING_WINDOW;
  dist4Avg /= ROLLING_WINDOW;

  // Cap distances at 80 cm
  dist1Avg = fmin(dist1Avg, 80.0);
  dist2Avg = fmin(dist2Avg, 80.0);
  dist3Avg = fmin(dist3Avg, 80.0);
  dist4Avg = fmin(dist4Avg, 80.0);

  // Normalize distances by dividing by 80
  float normDist1 = roundf((dist1Avg / 80.0) * 10000) / 10000;
  float normDist2 = roundf((dist2Avg / 80.0) * 10000) / 10000;
  float normDist3 = roundf((dist3Avg / 80.0) * 10000) / 10000;
  float normDist4 = roundf((dist4Avg / 80.0) * 10000) / 10000;

  // Print values every 0.1 seconds
  if (millis() - lastPrintTime >= 100) {
    Serial.print(heading);
    Serial.print(",");
    Serial.print(normDist1);
    Serial.print(",");
    Serial.print(normDist2);
    Serial.print(",");
    Serial.print(normDist3);
    Serial.print(",");
    Serial.println(normDist4);
    lastPrintTime = millis();
  }
}

// Function to get distance from ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343 / 2; // Convert to cm
  return distance;
}
