import serial
import serial.tools.list_ports
import time

def find_arduino_port():
    # Known vendor IDs for official and clone Arduino boards.
    known_vids = {0x2341, 0x1A86, 0x10C4}
    
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports detected on this system.")
        return None

    print("Available serial ports:")
    for port in ports:
        vid = hex(port.vid) if port.vid else "N/A"
        pid = hex(port.pid) if port.pid else "N/A"
        print(f" - {port.device}: {port.description} (VID: {vid}, PID: {pid})")
    
    # Search for ports with known vendor IDs.
    for port in ports:
        if port.vid and port.vid in known_vids:
            return port.device

    # Fallback: search the description for "arduino" (case insensitive).
    for port in ports:
        if "arduino" in port.description.lower():
            return port.device

    return None

def main():
    arduino_port = find_arduino_port()
    if not arduino_port:
        print("No Arduino device detected on any serial port.")
        return

    print(f"Arduino detected on port: {arduino_port}")
    
    try:
        with serial.Serial(arduino_port, 9600, timeout=1) as ser:
            time.sleep(2)  # Allow time for Arduino to reset after connection.
            print("Starting continuous ping. Press Ctrl+C to exit.")
            while True:
                try:
                    ser.write(b'P')  # Send the character "p".
                    time.sleep(0.5)  # Brief pause to allow the Arduino to respond.
                    response = ser.read(ser.inWaiting() or 1)
                    print("Arduino response:", response.decode().strip())
                    time.sleep(1)  # Wait before next ping.
                except KeyboardInterrupt:
                    print("\nExiting continuous ping.")
                    break
                except Exception as e:
                    print("An error occurred during ping:", e)
                    break
    except Exception as e:
        print("Error establishing serial connection:", e)

if __name__ == "__main__":
    main()
