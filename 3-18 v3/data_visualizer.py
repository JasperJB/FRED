# -*- coding: utf-8 -*-
import redis
import ast
import numpy as np
import time
import matplotlib.pyplot as plt

def main():
    # Connect to Redis
    rdb = redis.Redis(host='127.0.0.1', port=6379, db=0)

    plt.ion()  # Interactive mode so we can update in real-time
    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title("Rover Depth Visualization")

    # Initialize an empty 9Ã—12 image
    data = np.zeros((9, 12), dtype=np.float32)
    heatmap = ax.imshow(data, cmap="jet", vmin=0, vmax=1, origin="upper")
    cbar = plt.colorbar(heatmap, ax=ax)
    cbar.set_label("Normalized Distance (0..1)")

    ax.set_xticks(range(12))
    ax.set_yticks(range(9))

    # We'll display heading and recording state in the title
    ax.set_title("Heading: 0.00  |  Recording: Disabled")

    plt.tight_layout()

    try:
        while True:
            # 1) Retrieve camera_depth
            camera_val = rdb.get("camera_depth")
            if camera_val is not None:
                try:
                    arr_list = ast.literal_eval(camera_val.decode())
                    arr = np.array(arr_list, dtype=np.float32)
                    if arr.shape == (9, 12):
                        heatmap.set_data(arr)
                except:
                    pass

            # 2) Retrieve heading
            heading_val = rdb.get("arduino_heading")
            heading = 0.0
            if heading_val is not None:
                try:
                    heading = float(heading_val.decode())
                except:
                    pass

            # 3) Retrieve recording state
            rec_val = rdb.get("recording_enabled")
            recording_enabled = False
            if rec_val is not None:
                try:
                    recording_enabled = (int(rec_val.decode()) == 1)
                except:
                    pass

            # 4) Update the figure title
            heading_str = f"{heading:.2f}"
            rec_str = "Enabled" if recording_enabled else "Disabled"
            ax.set_title(f"Heading: {heading_str}  |  Recording: {rec_str}")

            # Redraw
            plt.draw()
            plt.pause(0.1)  # small delay to let the UI update
    except KeyboardInterrupt:
        print("Closing data_visualizer...")
    finally:
        plt.ioff()
        plt.close(fig)

if __name__ == "__main__":
    main()
