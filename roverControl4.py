from gpiozero import OutputDevice, PWMOutputDevice, DistanceSensor
from time import sleep

# Pin Definitions
ENA = PWMOutputDevice(12)  # PWM for right motors (ENA)
ENB = PWMOutputDevice(13)  # PWM for left motors (ENB)
IN1 = OutputDevice(17)  # Right motor forward
IN2 = OutputDevice(18)  # Right motor reverse
IN3 = OutputDevice(27)  # Left motor forward
IN4 = OutputDevice(22)  # Left motor reverse

# HC-SR04 Distance Sensor
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2.0)  # 2m max range

# Function to move forward
def move_forward(speed=0.5):  # Speed range: 0.0 to 1.0
    ENA.value = speed  # Set PWM duty cycle
    ENB.value = speed
    IN1.on()  # Right motors forward
    IN2.off()
    IN3.on()  # Left motors forward
    IN4.off()

# Function to stop motors
def stop_motors():
    ENA.value = 0
    ENB.value = 0
    IN1.off()
    IN2.off()
    IN3.off()
    IN4.off()

# Main Program
try:
    while True:
        distance = sensor.distance * 100  # Convert to cm
        print(f"Distance: {distance:.2f} cm")
        
        if distance < 10:  # Stop if obstacle < 10 cm
            print("Obstacle detected! Stopping motors.")
            stop_motors()
        else:
            print("Path clear. Moving forward.")
            move_forward(speed=0.5)  # Set motor speed to 50%

        sleep(0.1)  # Short delay

except KeyboardInterrupt:
    print("Program stopped by user.")
    stop_motors()
