#!/usr/bin/env python3
import asyncio
import threading
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6 import QtWidgets, QtCore

from sensor_streaming_handler import AccelerometerStream
from calculate_methods import estimate_trajectory_for_queue

# ---------------- CONFIG ----------------
IP = "172.21.11.52"
PORT = 8081
ARGS = "/sensor/connect?type=android.sensor.linear_acceleration"

WINDOW = 400        # sliding window samples
PLOT_HZ = 60        # UI update rate

prev_vx = 0
prev_vy = 0
prev_vz = 0
px, py, pz = 0, 0, 0
# ----------------------------------------

# ---------------- BUFFERS ----------------
timestamps = deque(maxlen=WINDOW)
ax_list = deque(maxlen=WINDOW)
ay_list = deque(maxlen=WINDOW)
az_list = deque(maxlen=WINDOW)
pts = np.array([])

lock = threading.Lock()
t0 = None
running = True
# -----------------------------------------


# ---------------- ASYNC SENSOR TASK ----------------
async def sensor_task():
    global t0, running

    t_ms = 0

    stream = AccelerometerStream(IP, PORT, ARGS)
    await stream.start()
    print("Accelerometer stream started")

    try:
        while running:
            data = await stream.get()
            ax, ay, az = data["values"]
            ts = data.get("timestamp")
            if ts is None:
                continue


            if t0 is not None:
                t_ms = (ts - t0) / 1e9  # ns → ms
            t0 = ts

            with lock:
                if t_ms > 0 and t_ms is not None:
                    timestamps.append(t_ms)
                ax_list.append(ax)
                ay_list.append(ay)
                az_list.append(az if (abs(az) > 0.9) else 0)
                print(az)

    finally:
        await stream.stop()


# ---------------- QT GRAPH ----------------
class TrajectoryViewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Live IMU Trajectory (PyQtGraph)")
        self.resize(900, 700)

        layout = QtWidgets.QVBoxLayout(self)
        self.view = gl.GLViewWidget()
        layout.addWidget(self.view)

        self.view.opts["distance"] = 20
        self.view.setBackgroundColor("k")

        # ---------- GRID ----------
        grid = gl.GLGridItem()
        grid.setSize(20, 20)
        grid.setSpacing(1, 1)
        self.view.addItem(grid)

        # ---------- AXES ----------
        axis_len = 5.0

        # X axis (red)
        self.view.addItem(
            gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [axis_len, 0, 0]]),
                color=(1, 0, 0, 1),
                width=3
            )
        )

        # Y axis (green)
        self.view.addItem(
            gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, axis_len, 0]]),
                color=(0, 1, 0, 1),
                width=3
            )
        )

        # Z axis (blue)
        self.view.addItem(
            gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], [0, 0, axis_len]]),
                color=(0, 0, 1, 1),
                width=3
            )
        )

        # ---------- AXIS LABELS ----------
        self.view.addItem(gl.GLTextItem(pos=(axis_len, 0, 0), text="X (m)", color=(1, 0, 0, 1)))
        self.view.addItem(gl.GLTextItem(pos=(0, axis_len, 0), text="Y (m)", color=(0, 1, 0, 1)))
        self.view.addItem(gl.GLTextItem(pos=(0, 0, axis_len), text="Z (m)", color=(0, 0, 1, 1)))

        # ---------- ORIGIN MARKER ----------
        self.view.addItem(
            gl.GLScatterPlotItem(
                pos=np.array([[0, 0, 0]]),
                size=10,
                color=(1, 1, 1, 1)
            )
        )

        # ---------- TRAJECTORY ----------
        self.line = gl.GLLinePlotItem(
            pos=np.zeros((1, 3)),
            width=2,
            antialias=True
        )
        self.view.addItem(self.line)

        # ---------- TIMER ----------
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(1000 / PLOT_HZ))

    def update_plot(self):
        with lock:
            if len(timestamps) < 10:
                return

            t = list(timestamps)
            axv = list(ax_list)
            ayv = list(ay_list)
            azv = list(az_list)

        # px, py, pz = estimate_trajectory(t, axv, ayv, azv)
        global pts
        px, py, pz, a, b, c = estimate_trajectory_for_queue(axv.pop(), ayv.pop(), azv.pop(), t.pop())
        pts = np.concatenate((pts, [px, py, pz]))

        # ---------- COLOR GRADIENT ----------
        n = len(pts)
        colors = np.zeros((n, 4), dtype=np.float32)
        colors[:, 0] = np.linspace(1.0, 0.0, n)   # red → 0
        colors[:, 1] = np.linspace(1.0, 1.0, n)   # green constant
        colors[:, 2] = np.linspace(0.0, 1.0, n)   # blue ↑
        colors[:, 3] = 1.0

        self.line.setData(pos=pts, color=colors)



# ---------------- MAIN ----------------
def main():
    global running

    # Start asyncio in background thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.create_task(sensor_task())

    t = threading.Thread(target=loop.run_forever, daemon=True)
    t.start()

    # Qt app (must be main thread)
    app = QtWidgets.QApplication([])
    viewer = TrajectoryViewer()
    viewer.show()

    try:
        app.exec()
    finally:
        running = False
        loop.call_soon_threadsafe(loop.stop)
        t.join()


if __name__ == "__main__":
    main()
 