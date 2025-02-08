#include <Wire.h>

// Define I2C addresses for MPU6050s
#define MPU1_ADDRESS 0x68
#define MPU2_ADDRESS 0x69

// Define pins for HC-SR04 sensors
#define TRIGGER_PIN_1 2
#define ECHO_PIN_1 3
#define TRIGGER_PIN_2 4
#define ECHO_PIN_2 5

// Initialize an MPU6050 device
void initializeMPU6050(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up the MPU6050
  Wire.endTransmission(true);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize both MPU6050 devices
  initializeMPU6050(MPU1_ADDRESS);
  initializeMPU6050(MPU2_ADDRESS);
}

void loop() {
  // Wait for a ping command from the Pi
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'P') {  // When the Pi pings, read sensors and respond
      int16_t ax1, ay1, az1, gx1, gy1, gz1;
      int16_t ax2, ay2, az2, gx2, gy2, gz2;
      
      readMPU6050(MPU1_ADDRESS, &ax1, &ay1, &az1, &gx1, &gy1, &gz1);
      readMPU6050(MPU2_ADDRESS, &ax2, &ay2, &az2, &gx2, &gy2, &gz2);
      
      long duration1 = measureDistance(TRIGGER_PIN_1, ECHO_PIN_1);
      long distance1 = duration1 * 0.034 / 2;
      long duration2 = measureDistance(TRIGGER_PIN_2, ECHO_PIN_2);
      long distance2 = duration2 * 0.034 / 2;
      
      // Send data in comma-separated format:
      Serial.print(ax1); Serial.print(",");
      Serial.print(ay1); Serial.print(",");
      Serial.print(az1); Serial.print(",");
      Serial.print(gx1); Serial.print(",");
      Serial.print(gy1); Serial.print(",");
      Serial.print(gz1); Serial.print(",");
      
      Serial.print(ax2); Serial.print(",");
      Serial.print(ay2); Serial.print(",");
      Serial.print(az2); Serial.print(",");
      Serial.print(gx2); Serial.print(",");
      Serial.print(gy2); Serial.print(",");
      Serial.print(gz2); Serial.print(",");
      
      Serial.print(distance1); Serial.print(",");
      Serial.println(distance2);
    }
  }
}

void readMPU6050(uint8_t address, int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(address);
  Wire.write(0x3B); // Start register for accelerometer data
  Wire.endTransmission(false); // Repeated start
  Wire.requestFrom(address, 14, true);
  
  *ax = Wire.read() << 8 | Wire.read();
  *ay = Wire.read() << 8 | Wire.read();
  *az = Wire.read() << 8 | Wire.read();
  // Skip temperature data
  Wire.read(); Wire.read();
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}

long measureDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH);
  return duration;
}
