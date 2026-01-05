#!/usr/bin/env python3
import asyncio
import json
import websockets


class AccelerometerStream:
    """
    Real-time WebSocket accelerometer stream.
    One network task, async-safe consumer interface.
    """

    def __init__(self, ip, port, args, queue_size=200):
        self.uri = f"ws://{ip}:{port}{args}"
        self.queue = asyncio.Queue(maxsize=queue_size)
        self._receiver_task = None
        self._running = False

    async def _receiver(self):
        """Internal task: receives data from WebSocket."""
        async with websockets.connect(self.uri) as ws:
            self._running = True
            while self._running:
                msg = await ws.recv()
                data = json.loads(msg)

                # Drop oldest data if queue is full (real-time behavior)
                if self.queue.full():
                    _ = self.queue.get_nowait()
                    self.queue.task_done()

                await self.queue.put(data)

    async def start(self):
        """
        Start the WebSocket receiver task.
        Call this ONCE from the owning event loop.
        """
        if self._receiver_task is None:
            self._receiver_task = asyncio.create_task(self._receiver())

    async def stop(self):
        """Gracefully stop the receiver."""
        self._running = False
        if self._receiver_task:
            self._receiver_task.cancel()
            try:
                await self._receiver_task
            except asyncio.CancelledError:
                pass

    async def get(self):
        """
        Get the next sensor sample.
        This blocks until new data arrives.
        """
        return await self.queue.get()

    def empty(self):
        return self.queue.empty()
