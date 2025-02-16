#!/usr/bin/env python3
import time
import sys
import select
import serial
import serial.tools.list_ports
from rich.console import Console
from rich.progress import track

console = Console()

def find_arduino():
    """
    Automatically detects the Arduino serial port.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if ("Arduino" in port.description) or ("ttyACM" in port.device) or ("ttyUSB" in port.device):
            return port.device
    raise RuntimeError("Arduino not detected.")

def parse_imu_data(line):
    """
    Parses a comma-separated string of 12 floats:
      Distance1,Distance2,Distance3,Distance4,Distance5,Distance6,
      avgAx,avgAy,avgAz,avgGx,avgGy,avgGz

    Returns (avgAx, avgAy, avgAz, avgGx, avgGy, avgGz)
    """
    parts = line.strip().split(',')
    if len(parts) != 12:
        return None
    try:
        values = list(map(float, parts))
        return tuple(values[6:12])
    except ValueError:
        return None

def calibrate_offsets(ser, num_samples=10):
    """
    Collects num_samples of IMU data and computes average offsets.
    Returns a tuple of offsets for (Ax, Ay, Az, Gx, Gy, Gz).
    """
    sums = [0.0] * 6
    valid = 0
    console.print("[bold yellow]Calibrating IMU offsets...[/bold yellow]")
    for _ in track(range(num_samples), description="Calibrating offsets"):
        ser.write(b'P')
        line = ser.readline().decode('utf-8').strip()
        data = parse_imu_data(line)
        if data is None:
            continue
        valid += 1
        for i in range(6):
            sums[i] += data[i]
        time.sleep(0.005)
    if valid == 0:
        raise RuntimeError("No valid IMU data during calibration.")
    offsets = tuple(s / valid for s in sums)
    console.print(f"[green]Offset calibration complete: {offsets}[/green]")
    return offsets

def check_for_enter():
    """
    Checks non-blockingly whether the user has pressed Enter.
    """
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        # read the input line to clear the buffer
        sys.stdin.readline()
        return True
    return False

def measure_rotation(ser, offset_z):
    """
    Integrates the gyro-z (after offset subtraction) until the user presses Enter.
    Returns the raw integrated angle (in degrees).
    """
    console.print("[bold cyan]Rotate now. Press Enter when finished.[/bold cyan]")
    integrated = 0.0
    last_time = time.time()
    while True:
        if check_for_enter():
            break
        ser.write(b'P')
        line = ser.readline().decode('utf-8').strip()
        data = parse_imu_data(line)
        if data is None:
            continue
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        gyro_z = data[5] - offset_z
        integrated += gyro_z * dt

    return integrated

def main():
    try:
        port = find_arduino()
        console.print(f"[bold cyan]Arduino detected on port: {port}[/bold cyan]")
    except RuntimeError as e:
        console.print(f"[bold red]{e}[/bold red]")
        return

    ser = serial.Serial(port, 9600, timeout=1)
    time.sleep(2)  # Allow Arduino to reset

    # Verify nonzero IMU data.
    console.print("[bold yellow]Verifying nonzero IMU data...[/bold yellow]")
    for _ in range(10):
        ser.write(b'P')
        line = ser.readline().decode('utf-8').strip()
        data = parse_imu_data(line)
        if data and any(abs(v) > 1e-6 for v in data):
            break
        time.sleep(0.005)
    else:
        console.print("[bold red]IMU data appears to be all zero. Check wiring or code.[/bold red]")
        ser.close()
        return

    # Calibrate offsets (focus on gyro-z offset).
    offsets = calibrate_offsets(ser, num_samples=10)
    offset_z = offsets[5]

    heading = 0.0
    console.print("[bold magenta]Heading set to 0°.[/bold magenta]")

    #
    # 1) Rotate +180° (right)
    #
    console.print("\n[bold cyan]Rotate the robot +180° (to the right).[/bold cyan]")
    input("Press Enter to begin measuring +180° rotation...")
    measured_right = measure_rotation(ser, offset_z)
    console.print(f"[green]Measured raw rotation: {measured_right:.2f}°[/green]")

    if abs(measured_right) < 1e-4:
        console.print("[bold red]No rotation detected. Aborting calibration.[/bold red]")
        ser.close()
        return

    # Scale so that measured_right => +180
    scale_right = 180.0 / measured_right
    scaled_right = measured_right * scale_right  # should be ~180
    console.print(f"[green]Scaling factor from right rotation: {scale_right:.4f}[/green]")
    # Check scaled value is close to 180
    if abs(scaled_right - 180.0) > 10.0:
        console.print("[bold red]Scaled +180° rotation not within ±10° of 180°. Calibration aborted.[/bold red]")
        ser.close()
        return

    # Reset heading
    heading = 0.0

    #
    # 2) Return to 0 by rotating +180° to the left
    #
    console.print("\n[bold cyan]Rotate the robot back to 0 (i.e., +180° to the left).[/bold cyan]")
    input("Press Enter to begin measuring return rotation...")
    measured_return = measure_rotation(ser, offset_z)
    console.print(f"[green]Measured raw return rotation: {measured_return:.2f}°[/green]")

    # We expect physically that the raw measurement is about -180
    # but we only check the *scaled* version:
    scaled_return = measured_return * scale_right
    console.print(f"[green]Scaled return rotation: {scaled_return:.2f}°[/green]")

    if abs(scaled_return + 180.0) > 10.0:
        console.print("[bold red]Scaled return rotation not within ±10° of -180°. Calibration aborted.[/bold red]")
        ser.close()
        return

    # A second scale factor from the return trip:
    scale_left = -180.0 / measured_return if abs(measured_return) > 1e-4 else scale_right
    console.print(f"[green]Secondary scaling factor from return: {scale_left:.4f}[/green]")

    #
    # 3) Rotate -180° (left)
    #
    console.print("\n[bold cyan]Rotate the robot -180° (to the left).[/bold cyan]")
    input("Press Enter to begin measuring -180° rotation...")
    measured_left = measure_rotation(ser, offset_z)
    console.print(f"[green]Measured raw left rotation: {measured_left:.2f}°[/green]")

    scaled_left = measured_left * scale_right  # scale using the first factor
    console.print(f"[green]Scaled left rotation: {scaled_left:.2f}°[/green]")

    if abs(scaled_left + 180.0) > 10.0:
        console.print("[bold red]Scaled -180° rotation not within ±10° of -180°. Calibration aborted.[/bold red]")
        ser.close()
        return

    # Another factor from the left rotation
    scale_left_2 = -180.0 / measured_left if abs(measured_left) > 1e-4 else scale_right
    console.print(f"[green]Third scaling factor from left rotation: {scale_left_2:.4f}[/green]")

    #
    # Combine all scale factors
    #
    final_scale = (scale_right + scale_left + scale_left_2) / 3.0
    console.print(f"[bold green]Final scaling factor: {final_scale:.4f}[/bold green]")

    #
    # 4) Final check: return to 0
    #
    console.print("\n[bold cyan]Rotate the robot back to 0.[/bold cyan]")
    input("Press Enter when the robot is near 0°...")
    console.print("[bold green]Calibration complete. Entering heading integration loop.[/bold green]")

    #
    # Main loop: integrate gyro-z with offset and final_scale
    #
    heading = 0.0
    last_time = time.time()
    try:
        while True:
            ser.write(b'P')
            line = ser.readline().decode('utf-8').strip()
            data = parse_imu_data(line)
            if data is None:
                continue

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            gyro_z = data[5] - offset_z
            heading += gyro_z * dt * final_scale
            heading %= 360.0

            console.print(f"[bold white]Heading:[/bold white] {heading:7.2f}°", end="\r")
    except KeyboardInterrupt:
        console.print("\n[bold red]Exiting...[/bold red]")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
