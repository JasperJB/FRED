#!/usr/bin/env python3
"""
This program interfaces with an Arduino via USB. Instead of the Arduino
pushing data continuously, the Pi pings the Arduino (sending 'P') so that:
  • The Arduino returns sensor data on demand.
  • The Pi records the exact time of each ping so that dt is computed accurately.
The program then performs a three‑step calibration (using 30‑second averages)
to determine an initial offset and drift rate. At runtime, each new reading is:
  1. Corrected by subtracting the offset (avg1) plus drift_rate * t.
  2. Converted from raw sensor units into physical units (accelerometer: m/s²,
     gyroscope: deg/s) using standard MPU6050 conversion factors.
  3. Integrated using dt computed from successive pings.
Rich loading bars are used during calibration.
"""

import time
import logging
import serial
import numpy as np
from rich.console import Console
from rich.progress import Progress

# Configure logging.
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s")

# Sensor axis keys (averaging left/right IMU data).
SENSOR_KEYS = ["ax", "ay", "az", "gx", "gy", "gz"]

# Conversion factors for MPU6050 (assumed default ±2g and ±250°/s ranges).
ACCEL_SCALE = 16384.0  # LSB per g
GRAVITY = 9.81         # m/s² per g
GYRO_SCALE = 131.0     # LSB per deg/s

def select_arduino_port():
    """
    Checks /dev/ttyACM0 and /dev/ttyACM1 for valid Arduino data.
    Returns the first port with plausible data.
    """
    possible_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]
    for port in possible_ports:
        try:
            ser = serial.Serial(port, baudrate=115200, timeout=1)
            time.sleep(2)  # Allow Arduino reset.
            ser.write(b'P')
            line = ser.readline().decode("utf-8").strip()
            if line.count(",") >= 13:
                logging.debug(f"Valid data detected on port {port}.")
                ser.close()
                return port
            ser.close()
        except Exception as e:
            logging.debug(f"Error accessing {port}: {e}")
    raise Exception("No valid Arduino port found among /dev/ttyACM0 and /dev/ttyACM1.")

def ping_and_read(ser):
    """
    Sends a ping ('P') to the Arduino and returns the parsed raw sensor data as a list of floats.
    """
    ser.write(b'P')
    line = ser.readline().decode("utf-8").strip()
    parts = line.split(',')
    if len(parts) < 14:
        return None
    try:
        return list(map(float, parts))
    except Exception:
        return None

def average_left_right(raw_values):
    """
    Averages the left/right IMU readings (first 12 values) and returns a dictionary with keys SENSOR_KEYS.
    """
    return {
        "ax": (raw_values[0] + raw_values[6]) / 2.0,
        "ay": (raw_values[1] + raw_values[7]) / 2.0,
        "az": (raw_values[2] + raw_values[8]) / 2.0,
        "gx": (raw_values[3] + raw_values[9]) / 2.0,
        "gy": (raw_values[4] + raw_values[10]) / 2.0,
        "gz": (raw_values[5] + raw_values[11]) / 2.0,
    }

def collect_average(ser, duration, label):
    """
    Repeatedly pings the Arduino for a given duration and computes the average of the left/right IMU values.
    """
    console = Console()
    sums = {key: 0.0 for key in SENSOR_KEYS}
    count = 0
    start = time.time()
    with Progress() as progress:
        task = progress.add_task(f"[cyan]{label}...", total=duration)
        while time.time() - start < duration:
            raw_values = ping_and_read(ser)
            if raw_values is None:
                continue
            avg_vals = average_left_right(raw_values)
            for key in SENSOR_KEYS:
                sums[key] += avg_vals[key]
            count += 1
            progress.update(task, advance=time.time() - start - progress.tasks[0].completed)
    if count > 0:
        return {key: sums[key] / count for key in SENSOR_KEYS}
    else:
        return {key: 0.0 for key in SENSOR_KEYS}

def calibration_sequence(ser):
    """
    Performs a three‑stage calibration:
      1. Collect avg1 (30 sec).
      2. Collect avg2 (30 sec) and verify that the relative difference is within 5% for all axes.
      3. Collect avg3 (30 sec) and compute drift_rate = (avg3 - avg2) / 30.
    Returns:
      calib_start_time: Absolute time when calibration started.
      initial_offset: The avg1 raw sensor values.
      drift_rate: Drift rate in raw units per second (dictionary).
    """
    console = Console()
    while True:
        console.print("[bold green]Starting calibration sequence...[/bold green]")
        calib_start_time = time.time()
        console.print("[cyan]Collecting initial 30-second average (avg1)...[/cyan]")
        avg1 = collect_average(ser, 30, "Collecting avg1")
        console.print(f"[yellow]Initial average (avg1):[/yellow] {avg1}")
        
        console.print("[cyan]Collecting second 30-second average (avg2)...[/cyan]")
        avg2 = collect_average(ser, 30, "Collecting avg2")
        console.print(f"[yellow]Second average (avg2):[/yellow] {avg2}")
        
        stable = True
        for key in SENSOR_KEYS:
            base = abs(avg1[key]) if abs(avg1[key]) > 1e-6 else 1e-6
            rel_diff = abs(avg2[key] - avg1[key]) / base
            if rel_diff > 0.05:
                stable = False
                console.print(f"[red]Calibration unstable for {key}: relative difference {rel_diff*100:.2f}% exceeds 5%.[/red]")
                break
        if not stable:
            console.print("[red]Repeating calibration sequence due to instability...[/red]")
            continue
        else:
            console.print("[green]Initial calibration is stable.[/green]")
            console.print("[cyan]Collecting additional 30-second data for drift measurement (avg3)...[/cyan]")
            avg3 = collect_average(ser, 30, "Collecting avg3")
            console.print(f"[yellow]Drift average (avg3):[/yellow] {avg3}")
            drift_rate = {}
            for key in SENSOR_KEYS:
                drift_rate[key] = (avg3[key] - avg2[key]) / 30.0
                console.print(f"[blue]{key} drift rate: {drift_rate[key]:.6f} per second[/blue]")
            break
    return calib_start_time, avg1, drift_rate

def main():
    console = Console()
    try:
        port = select_arduino_port()
        logging.debug(f"Selected Arduino port: {port}")
    except Exception as e:
        logging.error(f"Port selection failed: {e}")
        return

    try:
        ser = serial.Serial(port, baudrate=115200, timeout=1)
        ser.setDTR(False)
        time.sleep(0.1)
        ser.setDTR(True)
        time.sleep(2)
    except Exception as e:
        logging.error(f"Failed to open serial port {port}: {e}")
        return

    calib_start_time, initial_offset, drift_rate = calibration_sequence(ser)
    velocity = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
    heading = 0.0
    prev_ping_time = time.time()
    
    console.print("[bold blue]Entering main loop. Press Ctrl+C to terminate.[/bold blue]")
    try:
        while True:
            current_ping_time = time.time()
            dt = current_ping_time - prev_ping_time
            prev_ping_time = current_ping_time
            
            raw_values = ping_and_read(ser)
            if raw_values is None:
                continue
            
            # Print velocity with units
            console.print(f"[yellow]Velocity:[/yellow] Vx={velocity['vx']:.2f} m/s, Vy={velocity['vy']:.2f} m/s, Vz={velocity['vz']:.2f} m/s")
            console.print(f"[yellow]Heading:[/yellow] {heading:.2f}°")
    except KeyboardInterrupt:
        logging.info("Termination requested by user.")
