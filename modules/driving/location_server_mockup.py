#!/usr/bin/env python

# WS server example

import asyncio
import websockets

PLACES = {"mensa", "mi", "mw", "ubahn"}

async def server(websocket, path):
    location = "mensa"
    while(True):
        await websocket.send(location)
        eta = await websocket.recv()
        print(eta)

start_server = websockets.serve(server, "localhost", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()