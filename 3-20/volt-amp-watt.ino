#include <math.h>  // Needed for fabs()

const int voltagePin = A0;  // Voltage sensor output
const int currentPin = A1;  // ACS712 output

const float vRef = 5.0;     
const float voltageDividerRatio = 4.72;
const float ACS712_Sensitivity = 0.066;  // For ACS712-5A

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Read and calculate the measured voltage
    int rawVoltage = analogRead(voltagePin);
    float measuredVoltage = (rawVoltage * vRef / 1023.0) * voltageDividerRatio;
    
    // Read and calculate the measured current
    int rawCurrent = analogRead(currentPin);
    float voltageOffset = 2.5;  // ACS712 outputs ~2.5V at 0A
    float measuredCurrent = ((rawCurrent * vRef / 1023.0) - voltageOffset) / ACS712_Sensitivity;

    // Force the reading to be nonnegative
    measuredCurrent = fabs(measuredCurrent);
    
    // Calculate power
    float power = measuredVoltage * measuredCurrent;

    // Print results
    Serial.print("Voltage (V): ");
    Serial.print(measuredVoltage);
    Serial.print(" | Current (A): ");
    Serial.print(measuredCurrent);
    Serial.print(" | Power (W): ");
    Serial.println(power);

    delay(1000);
}
