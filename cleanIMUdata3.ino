#include <Wire.h>

#define IMU1_ADDRESS 0x68
#define IMU2_ADDRESS 0x69
#define SLAVE_ADDRESS 8

#define TRIG_PIN 5
#define ECHO_PIN_1 2
#define ECHO_PIN_2 3
#define ECHO_PIN_3 4
#define ECHO_PIN_4 6
#define ECHO_PIN_5 7
#define ECHO_PIN_6 8

float roll_avg[3] = {0, 0, 0}; // Rolling averages for x, y, z
float heading = 0; // Heading in degrees
float velocity[2] = {0, 0}; // Velocity in x, y directions (m/s)
float position[2] = {0, 0}; // Position in x, y coordinates (m)

const int AVG_WINDOW = 5;
float avg_buffer[3][AVG_WINDOW] = {{0}};
int avg_index = 0;

unsigned long last_time = 0;
const float dt = 0.02; // Sampling time (50 Hz)

uint16_t distances[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize I2C for Raspberry Pi communication (slave mode)
  Wire.begin(SLAVE_ADDRESS); // Default I2C pins (SDA/SCL)
  Wire.onRequest(requestEvent); // Register request event handler
  Serial.println("I2C initialized for Raspberry Pi communication (Slave mode).");

  // Initialize I2C for IMU communication (master mode) on custom pins
  Wire1.begin(A3, A0); // A3 as SDA, A0 as SCL
  Wire1.setClock(100000); // Set I2C clock speed to 100kHz
  Serial.println("I2C initialized for IMU communication on A3 (SDA) and A0 (SCL).");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(ECHO_PIN_3, INPUT);
  pinMode(ECHO_PIN_4, INPUT);
  pinMode(ECHO_PIN_5, INPUT);
  pinMode(ECHO_PIN_6, INPUT);

  initializeIMU(IMU1_ADDRESS);
  initializeIMU(IMU2_ADDRESS);

  last_time = millis();
}

void initializeIMU(uint8_t address) {
  Wire1.beginTransmission(address);
  Wire1.write(0x6B); // Power management register
  Wire1.write(0x00); // Wake up the sensor
  Wire1.endTransmission();

  Wire1.beginTransmission(address);
  Wire1.write(0x1C); // Accelerometer configuration
  Wire1.write(0x00); // Set to ±2g range
  Wire1.endTransmission();
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_time >= dt * 1000) {
    last_time = current_time;

    // Read IMU data
    float accel1[3], accel2[3];
    readIMU(IMU1_ADDRESS, accel1);
    readIMU(IMU2_ADDRESS, accel2);

    // Average the two IMU data sets
    float avg_accel[3];
    for (int i = 0; i < 3; i++) {
      avg_accel[i] = (accel1[i] + accel2[i]) / 2.0;
    }

    // Update rolling averages
    for (int i = 0; i < 3; i++) {
      avg_buffer[i][avg_index] = avg_accel[i];
      roll_avg[i] = 0;
      for (int j = 0; j < AVG_WINDOW; j++) {
        roll_avg[i] += avg_buffer[i][j];
      }
      roll_avg[i] /= AVG_WINDOW;
    }
    avg_index = (avg_index + 1) % AVG_WINDOW;

    // Compute heading (assume roll_avg[0] is x and roll_avg[1] is y acceleration)
    heading = atan2(roll_avg[1], roll_avg[0]) * 180 / PI;

    // Integrate acceleration to update velocity and position
    for (int i = 0; i < 2; i++) {
      velocity[i] += roll_avg[i] * dt;
      position[i] += velocity[i] * dt;
    }

    // Update distance sensor data
    updateDistances();
  }
}

void readIMU(uint8_t address, float *accel) {
  Wire1.beginTransmission(address);
  Wire1.write(0x3B); // Starting register for accelerometer data
  Wire1.endTransmission(false);
  Wire1.requestFrom(address, 6); // Request 6 bytes (X, Y, Z high/low)

  for (int i = 0; i < 3; i++) {
    int16_t raw = (Wire1.read() << 8) | Wire1.read();
    accel[i] = raw / 16384.0; // Convert to g (assuming ±2g range)
  }
}

void updateDistances() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  distances[0] = measureDistance(ECHO_PIN_1);
  distances[1] = measureDistance(ECHO_PIN_2);
  distances[2] = measureDistance(ECHO_PIN_3);
  distances[3] = measureDistance(ECHO_PIN_4);
  distances[4] = measureDistance(ECHO_PIN_5);
  distances[5] = measureDistance(ECHO_PIN_6);
}

uint16_t measureDistance(int echoPin) {
  unsigned long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  return duration * 0.034 / 2; // Convert to cm
}

void requestEvent() {
  uint8_t buffer[24 + 12];  // Buffer to hold processed IMU and sensor data

  // Pack position data into the buffer
  int index = 0;
  for (int i = 0; i < 2; i++) {
    int16_t coord = position[i] * 100; // Scale to int16 for transmission
    buffer[index++] = (coord >> 8) & 0xFF; // High byte
    buffer[index++] = coord & 0xFF;       // Low byte
  }

  // Pack heading data into the buffer
  int16_t head = heading * 100; // Scale to int16 for transmission
  buffer[index++] = (head >> 8) & 0xFF; // High byte
  buffer[index++] = head & 0xFF;       // Low byte

  // Pack velocity data into the buffer
  for (int i = 0; i < 2; i++) {
    int16_t vel = velocity[i] * 100; // Scale to int16 for transmission
    buffer[index++] = (vel >> 8) & 0xFF; // High byte
    buffer[index++] = vel & 0xFF;       // Low byte
  }

  // Pack ultrasonic sensor data into the buffer
  for (int i = 0; i < 6; i++) {
    buffer[index++] = (distances[i] >> 8) & 0xFF;  // High byte
    buffer[index++] = distances[i] & 0xFF;         // Low byte
  }

  // Send the buffer to the Raspberry Pi
  Wire.write(buffer, sizeof(buffer));
}
