const int echo = 11;
const int trig = 12;
const int NUM_SAMPLES = 5; // Number of samples for averaging

void setup() {
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  
  Serial.begin(9600); // Ensure Serial Monitor is set to 9600
}

float get_distance() {
  long echo_time;

  // Send a 10-microsecond trigger pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Measure the echo time with a timeout
  echo_time = pulseIn(echo, HIGH, 30000); // 30ms timeout

  if (echo_time == 0) {
    return -1; // Indicate no reading (out of range or no object detected)
  }

  // Calculate distance in cm
  return (echo_time / 2.0) * (343.0 / 10000.0);
}

float get_filtered_distance() {
  float distances[NUM_SAMPLES];
  int valid_samples = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    float d = get_distance();
    if (d > 0) { // Ignore invalid readings
      distances[valid_samples++] = d;
    }
    delay(50); // Small delay between samples
  }

  if (valid_samples == 0) return -1; // If no valid readings, return -1

  // Sort the valid samples
  for (int i = 0; i < valid_samples - 1; i++) {
    for (int j = i + 1; j < valid_samples; j++) {
      float temp = distances[i];
      if (distances[i] > temp) {
        
        distances[i] = temp;
        
      }
    }
  }

  return distances[valid_samples / 2]; // Return median of valid samples
}

void loop() {
  float distance = get_filtered_distance();

  Serial.print("Filtered Distance: ");
  if (distance == -1) {
    Serial.println("No object detected");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }

}
