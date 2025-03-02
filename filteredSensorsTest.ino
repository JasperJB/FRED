#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pin assignments for five ultrasonic sensors (Trig, Echo)
const int trigPins[5] = {13, 11, 9, 7, 3};
const int echoPins[5] = {12, 10, 8, 6, 2};

const int NUM_SAMPLES = 5; // Number of samples for filtering
P

// Create two IMU objects
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;

// Variables for heading calculation
float heading = 0.0f;  // Integrated heading in degrees
float gzOffset = 0.0f;  // Gyro Z offset in degrees/sec
unsigned long prevTime = 0; // For integration time step

// Function to measure distance (in cm) with filtering
float get_filtered_distance(int trigPin, int echoPin) {
    float distances[NUM_SAMPLES];
    int valid_samples = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
        float distance = duration * 0.0343f / 2.0f;

        if (distance > 0) { // Ignore invalid readings
            distances[valid_samples++] = distance;
        }
        delay(50); // Small delay between samples
    }

    if (valid_samples == 0) return -1; // No valid readings

    // Sort the valid samples for median filtering
    for (int i = 0; i < valid_samples - 1; i++) {
        for (int j = i + 1; j < valid_samples; j++) {
            if (distances[i] > distances[j]) {
                float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    return distances[valid_samples / 2]; // Return median value
}

void setup() {
    Serial.begin(9600);

    // Set ultrasonic sensor pins
    for (int i = 0; i < 5; i++) {
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

    Serial.println("Calibrated");

    prevTime = millis();
}

void loop() {
    // Integrate heading continuously
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
            // Read distances from all five ultrasonic sensors
            float distances[5];

            for (int i = 0; i < 5; i++) {
                distances[i] = get_filtered_distance(trigPins[i], echoPins[i]);
            }

            // Print data in comma-separated format:
            // 1) Ultrasonic distances
            for (int i = 0; i < 5; i++) {
                Serial.print(distances[i], 2);
                Serial.print(",");
            }

            // 2) Current heading
            Serial.print(heading, 2);
            Serial.println(); // End line
        }
    }
}
