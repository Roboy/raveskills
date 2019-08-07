#!/usr/bin/env python

# WS client example

import asyncio
import websockets
import pickle

async def hello():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        var = await websocket.recv()
        var2 = pickle.loads(var)
        return var2

var = asyncio.get_event_loop().run_until_complete(hello())
print(var)