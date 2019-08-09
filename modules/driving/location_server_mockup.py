#!/usr/bin/env python

# WS server example

import asyncio
import websockets
import face_recognition
import pickle


PLACES = {"mensa", "mi", "mw", "ubahn"}

async def server(websocket, path):
    eta = await websocket.recv()
    print(eta)



if __name__ == '__main__':
    start_server = websockets.serve(server, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()