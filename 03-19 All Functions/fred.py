# -*- coding: utf-8 -*-
import subprocess
import time
import redis
from rich.console import Console
from rich.panel import Panel

console = Console()

def main():
    console.print(Panel.fit("[bold magenta]Starting up your rover processes...[/bold magenta]"))

    # Ensure recording is disabled at startup
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)
    r.set("recording_enabled", 0)

    # 1) Motor Control (hide logs to avoid spam)
    console.print("[blue]Launching [bold]motorcontrol.py[/bold]...[/blue]")
    motor_process = subprocess.Popen(
        ["python3", "motorcontrol.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)

    # 2) PS4 Control (show logs for button press feedback)
    console.print("[blue]Launching [bold]control.py[/bold]...[/blue]")
    control_process = subprocess.Popen(["python3", "control.py"])

    # 3) Arduino Data (show logs for heading and reset confirmation)
    console.print("[blue]Launching [bold]arduino_data.py[/bold]...[/blue]")
    arduino_process = subprocess.Popen(["python3", "arduino_data.py"])
    time.sleep(2)

    # 4) Camera Data (hide logs to avoid spam)
    console.print("[blue]Launching [bold]camera_data.py[/bold]...[/blue]")
    camera_process = subprocess.Popen(
        ["python3", "camera_data.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # 5) Data Collection (hide logs to avoid spam)
    console.print("[blue]Launching [bold]data_collection.py[/bold]...[/blue]")
    data_collection_process = subprocess.Popen(
        ["python3", "data_collection.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # Ask user if they want to launch data_visualizer.py
    console.print("[bold yellow]Would you like to launch data_visualizer.py? (y/n)[/bold yellow]")
    answer = input("> ").strip().lower()
    data_visualizer_process = None
    if answer == "y":
        console.print("[blue]Launching [bold]data_visualizer.py[/bold]...[/blue]")
        data_visualizer_process = subprocess.Popen(["python3", "data_visualizer.py"])

    # Final instructions to the user
    console.print(
        Panel.fit(
            "[bold green]All processes have started successfully![/bold green]\n\n"
            "[cyan]Press [bold]Circle (O)[/bold] on PS4 controller to start recording.[/cyan]\n"
            "[cyan]Press [bold]Cross (X)[/bold] to stop recording.[/cyan]\n"
            "[cyan]Press [bold]Triangle[/bold] to reset Arduino.[/cyan]\n"
            "[yellow]Press [bold]CTRL + C[/bold] to shut everything down.[/yellow]"
        )
    )

    # Keep the main thread alive until user terminates (Ctrl+C)
    try:
        motor_process.wait()
        control_process.wait()
        arduino_process.wait()
        camera_process.wait()
        data_collection_process.wait()
        if data_visualizer_process:
            data_visualizer_process.wait()
    except KeyboardInterrupt:
        console.print("[bold red]\nStopping all processes...[/bold red]")
        motor_process.terminate()
        control_process.terminate()
        arduino_process.terminate()
        camera_process.terminate()
        data_collection_process.terminate()
        if data_visualizer_process:
            data_visualizer_process.terminate()

if __name__ == "__main__":
    main()
