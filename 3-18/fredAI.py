# -*- coding: utf-8 -*-
import subprocess
import time

from rich.console import Console
from rich.panel import Panel

console = Console()

def main():
    console.print(Panel.fit("[bold magenta]Starting up your AI-based rover processes...[/bold magenta]"))

    # 1) Motor Control (hide logs to avoid spam)
    console.print("[blue]Launching [bold]motorcontrol.py[/bold]...[/blue]")
    motor_process = subprocess.Popen(
        ["python3", "motorcontrol.py"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)

    # 2) AI Inference Script (Camera + Heading)
    console.print("[blue]Launching [bold]inference_ai.py[/bold]...[/blue]")
    inference_process = subprocess.Popen(["python3", "inference_ai.py"])

    # 3) Arduino Data
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

    # Optionally launch data visualizer
    console.print("[bold yellow]Would you like to launch data_visualizer.py? (y/n)[/bold yellow]")
    answer = input("> ").strip().lower()
    data_visualizer_process = None
    if answer == "y":
        console.print("[blue]Launching [bold]data_visualizer.py[/bold]...[/blue]")
        data_visualizer_process = subprocess.Popen(["python3", "data_visualizer.py"])

    console.print(
        Panel.fit(
            "[bold green]All processes have started with AI (camera+heading) inference![/bold green]\n\n"
            "[cyan]Your TFLite model is now controlling the motors based on camera depth & heading.[/cyan]\n"
            "[yellow]Press [bold]CTRL + C[/bold] to shut everything down.[/yellow]"
        )
    )

    try:
        motor_process.wait()
        inference_process.wait()
        arduino_process.wait()
        camera_process.wait()
        data_collection_process.wait()
        if data_visualizer_process:
            data_visualizer_process.wait()
    except KeyboardInterrupt:
        console.print("[bold red]\nStopping all processes...[/bold red]")
        motor_process.terminate()
        inference_process.terminate()
        arduino_process.terminate()
        camera_process.terminate()
        data_collection_process.terminate()
        if data_visualizer_process:
            data_visualizer_process.terminate()

        motor_process.wait()
        inference_process.wait()
        arduino_process.wait()
        camera_process.wait()
        data_collection_process.wait()
        if data_visualizer_process:
            data_visualizer_process.wait()

if __name__ == "__main__":
    main()
