import RPi.GPIO as GPIO
import redis

MOTOR_PINS = {
    "ENA": 12,
    "ENB": 13,
    "IN1": 17,
    "IN2": 18,
    "IN3": 27,
    "IN4": 22,
}

GPIO.setmode(GPIO.BCM)
GPIO.setup(list(MOTOR_PINS.values()), GPIO.OUT)
GPIO.output(MOTOR_PINS["ENA"], GPIO.HIGH)
GPIO.output(MOTOR_PINS["ENB"], GPIO.HIGH)

def set_motor_state(in1, in2, in3, in4):
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH if in1 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH if in2 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH if in3 else GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH if in4 else GPIO.LOW)

def main():
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    pubsub = r.pubsub()
    pubsub.subscribe("motor_command")  
    print("Motor control subscribed to Redis channel 'motor_command'...")

    try:
        for message in pubsub.listen():
            if message["type"] == "message":
                cmd = message["data"].decode()  # e.g. "1,0,1,0"
                if not cmd:
                    continue
                in1, in2, in3, in4 = map(int, cmd.split(","))
                set_motor_state(in1, in2, in3, in4)
    except KeyboardInterrupt:
        print("Shutting down motor control.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
