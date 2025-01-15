#include <Wire.h>

// --- Ultrasonic sensor pins ---
const int TRIG_PIN = 5;
const int ECHO_PINS[6] = {2, 3, 4, 6, 7, 8};

// --- MPU6050 addresses ---
#define MPU1_ADDR 0x68
#define MPU2_ADDR 0x69

// We'll store orientation/gyro data here
float yaw   = 0.0;
float pitch = 0.0;
float roll  = 0.0;

// Very rough “global” position estimate
float posX = 0.0;
float posY = 0.0;
float velocity = 0.0;

// Stubs to store motor commands (we'll get them from the Pi or an internal logic)
float leftMotorCmd = 0.0;   // range -1 to 1
float rightMotorCmd = 0.0;  // range -1 to 1

// For a simple integration approach
unsigned long prevTime = 0;
float dt = 0.0;

void setup() {
  Serial.begin(115200);

  // Configure ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  for (int i = 0; i < 6; i++) {
    pinMode(ECHO_PINS[i], INPUT);
  }

  // Initialize I2C
  Wire.begin(0x10);  // Arduino Nano ESP32 as I2C slave, address 0x10 (arbitrary)
  Wire.onRequest(requestEvent);

  // Initialize MPU6050s
  initMPU(MPU1_ADDR);
  initMPU(MPU2_ADDR);

  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0; 
  prevTime = currentTime;

  // 1) Read ultrasonic distances
  float distances[6];
  for (int i = 0; i < 6; i++) {
    distances[i] = readUltrasonic(ECHO_PINS[i]);
  }

  // 2) Read from MPU6050 #1 (accel/gyro).
  //    For real-world usage, you'd do the same for MPU #2 and maybe fuse them.
  float gx, gy, gz, ax, ay, az;
  readMPU(MPU1_ADDR, gx, gy, gz, ax, ay, az);

  // 3) Simple yaw integration from gyro data (gz) -- not true heading!
  //    gz is in degrees/sec if configured that way. 
  yaw += gz * dt;

  // 4) Very naive “velocity” and “position” update from accelerometer.
  //    (In real code, you’d do proper sensor fusion.)
  //    Suppose ax is in m/s^2, just integrate for velocity (again, naive).
  velocity += ax * dt;
  float distanceTraveled = velocity * dt;
  // Arbitrarily treat yaw as heading, move in that direction
  posX += distanceTraveled * cos(radians(yaw));
  posY += distanceTraveled * sin(radians(yaw));

  // 5) [Optional] read the second MPU (MPU2_ADDR) if you want data from both

  // 6) Keep everything else running ...
  delay(50);
}

// --------------------- Helper Functions ---------------------

void initMPU(uint8_t address) {
  Wire.beginTransmission(address);
  // Wake up MPU6050
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Optionally configure gyro and accel ranges here
}

void readMPU(uint8_t address, float &gx, float &gy, float &gz,
             float &ax, float &ay, float &az) {
  // Example reading raw registers for gyro and accel
  // This is greatly simplified; you would typically have 14 bytes to read:
  // 0x3B..0x40 => accelX, accelY, accelZ, temp
  // 0x43..0x48 => gyroX, gyroY, gyroZ
  // You’d convert from raw to real units based on your config.
  
  Wire.beginTransmission(address);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)14, (uint8_t)true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); // skip temp high
  Wire.read(); // skip temp low
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert to "units" -- these scale factors depend on config. 
  ax = rawAx / 16384.0; 
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;

  gx = rawGx / 131.0;  
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;
}

// Triggers ultrasonic sensor on TRIG_PIN, then reads a single ECHO pin.
float readUltrasonic(int echoPin) {
  // Send a 10us pulse on TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo
  long duration = pulseIn(echoPin, HIGH, 30000UL); // microseconds
  if (duration == 0) {
    // no pulse, out of range or error
    return -1.0;
  }
  
  // convert to distance in cm (speed of sound ~ 340 m/s => 29.1 us/cm round-trip)
  float distanceCm = duration * 0.0343 / 2.0;
  return distanceCm;
}

// I2C request handler
void requestEvent() {
  // Construct your CSV string
  // [leftMotorCmd, rightMotorCmd, posX, posY, velocity, heading, dist1...dist6]
  // For brevity, we’ll re-measure distances here (though you’d normally store them from loop()):

  float distances[6];
  for (int i = 0; i < 6; i++) {
    distances[i] = readUltrasonic(ECHO_PINS[i]);
  }

  // Build CSV string
  String out = String(leftMotorCmd, 3) + "," +
               String(rightMotorCmd, 3) + "," +
               String(posX, 3)        + "," +
               String(posY, 3)        + "," +
               String(velocity, 3)    + "," +
               String(yaw, 3);
  for (int i = 0; i < 6; i++) {
    out += ",";
    out += String(distances[i], 2);
  }

  // Send string back to Pi
  Wire.write(out.c_str());
}

