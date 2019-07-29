#!/usr/bin/env python

# WS client example

import asyncio
import websockets

async def hello():
    uri = "ws://localhost:8766"
    async with websockets.connect(uri) as websocket:
        location = await websocket.recv()
        return location

location = asyncio.get_event_loop().run_until_complete(hello())
print(location)