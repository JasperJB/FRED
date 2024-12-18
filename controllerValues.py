import evdev
from evdev import InputDevice, categorize, ecodes

# Find the PS4 controller among input devices
def find_ps4_controller():
    devices = [InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if 'Wireless Controller' in device.name:
            return device
    return None

def map_value(value, in_min, in_max, out_min, out_max):
    # Map the joystick values from one range to another
    mapped_value = int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    # Reverse the direction of values
    mapped_value = -mapped_value
    # Set a dead zone around 0
    if -10 <= mapped_value <= 10:
        return 0
    return mapped_value

def main():
    ps4_controller = find_ps4_controller()

    if not ps4_controller:
        print("PS4 controller not found. Make sure it is connected via Bluetooth.")
        return

    print(f"Connected to {ps4_controller.name}")

    # Initialize variables for joystick values
    left_y = 0
    right_y = 0

    # Loop to read events from the controller
    try:
        for event in ps4_controller.read_loop():
            if event.type == ecodes.EV_ABS:
                absevent = categorize(event)
                # Check for left joystick y-axis (ABS_Y) and right joystick y-axis (ABS_RY)
                if event.code == ecodes.ABS_Y:
                    left_y = map_value(absevent.event.value, 0, 255, -255, 255)
                elif event.code == ecodes.ABS_RY:
                    right_y = map_value(absevent.event.value, 0, 255, -255, 255)
                print(f"left: {left_y}, right: {right_y}")
    except KeyboardInterrupt:
        print("Exiting...")

if __name__ == "__main__":
    main()
