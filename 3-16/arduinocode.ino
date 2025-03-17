#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define NUM_SENSORS 4
#define ROLLING_AVG_SIZE 5

// Ultrasonic sensor pins (Trig, Echo)
const int trigPins[NUM_SENSORS] = {13, 9, 7, 3};
const int echoPins[NUM_SENSORS] = {12, 8, 6, 2};

// Rolling average buffers
float distanceBuffer[NUM_SENSORS][ROLLING_AVG_SIZE] = {0};
int bufferIndex[NUM_SENSORS] = {0};

// IMU setup
Adafruit_MPU6050 mpu1, mpu2;
float heading = 0;
float angularVelocityZ = 0;
float lastTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize MPU6050 IMUs
    if (!mpu1.begin(0x68)) {
        Serial.println("Failed to find MPU6050 #1");
        while (1);
    }
    if (!mpu2.begin(0x69)) {
        Serial.println("Failed to find MPU6050 #2");
        while (1);
    }

    // Set IMU sample rate and filtering
    mpu1.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu1.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu1.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu2.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Set pin modes for ultrasonic sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    lastTime = millis() / 1000.0;  // Initialize time
}

// Function to get distance from an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);  // Timeout after 30ms
    float distance = duration * 0.0343 / 2;  // Convert to cm
    return (distance > 400) ? 400 : distance; // Cap max range to 400cm
}

// Function to maintain a rolling average
float rollingAverage(float newValue, float buffer[], int &index) {
    buffer[index] = newValue;
    index = (index + 1) % ROLLING_AVG_SIZE;

    float sum = 0;
    for (int i = 0; i < ROLLING_AVG_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / ROLLING_AVG_SIZE;
}

// Function to calculate heading from IMU data
void updateHeading() {
    float currentTime = millis() / 1000.0;
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;

    sensors_event_t a1, g1, temp1, a2, g2, temp2;
    mpu1.getEvent(&a1, &g1, &temp1);
    mpu2.getEvent(&a2, &g2, &temp2);

    // Average the Z-axis gyro readings from both IMUs
    angularVelocityZ = (g1.gyro.z + g2.gyro.z) / 2.0;

    // Integrate angular velocity to get approximate heading
    heading += angularVelocityZ * deltaTime;
}

void loop() {
    float distances[NUM_SENSORS];

    // Read ultrasonic sensors one at a time
    for (int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = getDistance(trigPins[i], echoPins[i]);
        distances[i] = rollingAverage(distances[i], distanceBuffer[i], bufferIndex[i]);
        delay(50); // Short delay to reduce interference
    }

    // Update heading from IMU data
    updateHeading();

    // Print results
    Serial.print(distances[0]); Serial.print(",");
    Serial.print(distances[1]); Serial.print(",");
    Serial.print(distances[2]); Serial.print(",");
    Serial.print(distances[3]); Serial.print(",");
    Serial.println(heading);

    delay(50);  // Allow time for stable readings
}
