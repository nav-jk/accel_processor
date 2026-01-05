#!/usr/bin/env python3
import asyncio
import json
import time
import websockets
import pandas as pd

HOST = "0.0.0.0"
PORT = 8080
EXPECTED_PATH = "/sensor/connect"

CSV_PATH = "./expt_results/Linear Acceleration.csv"
REPLAY_SPEED = 1.0   # 1.0 = real time, 2.0 = 2× faster, 0.5 = slower


def load_csv():
    df = pd.read_csv(CSV_PATH)

    # ---- Time column is ALREADY seconds ----
    t = df["Time (s)"].astype(float).to_numpy()
    t = t - t[0]   # normalize to start at 0

    ax = df["Linear Acceleration x (m/s^2)"].astype(float).to_numpy()
    ay = df["Linear Acceleration y (m/s^2)"].astype(float).to_numpy()
    az = df["Linear Acceleration z (m/s^2)"].astype(float).to_numpy()

    return t, ax, ay, az


async def sensor_handler(ws):
    path = ws.request.path
    if not path.startswith(EXPECTED_PATH):
        print(f"Rejected connection on path: {path}")
        await ws.close()
        return

    print(f"Client connected on {path}")

    # Load CSV once per client
    t, ax, ay, az = load_csv()

    t_ns_start = time.time_ns()

    try:
        for i in range(len(t)):
            if i > 0:
                dt = (t[i] - t[i - 1]) / REPLAY_SPEED
                if dt > 0:
                    await asyncio.sleep(dt)

            payload = {
                "sensor": "android.sensor.accelerometer",
                "values": [float(ax[i]), float(ay[i]), float(az[i])],
                "timestamp": t_ns_start + int(t[i] * 1e9)
            }

            await ws.send(json.dumps(payload))

        print("CSV replay finished")

    except websockets.ConnectionClosed:
        print("Client disconnected")


async def main():
    async with websockets.serve(sensor_handler, HOST, PORT):
        print(f"CSV sensor server running on ws://{HOST}:{PORT}")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
