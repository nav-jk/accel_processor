IP = "172.21.11.52"
PORT = 8080
ARGS = "/sensor/connect?type=android.sensor.accelerometer"

#!/usr/bin/env python
"""Client using the asyncio API."""

import asyncio
import json
import websockets

queue = asyncio.Queue(maxsize=100)

async def receiver(uri):
    async with websockets.connect(uri) as ws:
        while True:
            msg = await ws.recv()
            data = json.loads(msg)
            await queue.put(data)  # fast handoff

async def processor():
    while True:
        data = await queue.get()

        # Example processing
        # sensor = data.get("sensor")
        value = data.get("values")
        print(f"Processing {value[0]}")

        queue.task_done()

async def main():
    uri = f"ws://{IP}:{PORT}{ARGS}"
    print(uri)
    await asyncio.gather(
        receiver(uri),
        processor(),
    )

if __name__ == "__main__":
    asyncio.run(main())
