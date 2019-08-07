#!/usr/bin/env python

# WS server example

import asyncio
import websockets
import face_recognition
import pickle


PLACES = {"mensa", "mi", "mw", "ubahn"}

async def server(websocket, path):
    #location = "ubahn"
    #await websocket.send(location)
    known_image = face_recognition.load_image_file("biden.jpg")
    biden_encoding = face_recognition.face_encodings(known_image)[0]
    print(biden_encoding)
    pickled_encodings = pickle.dumps((biden_encoding, bytes(), "abc"), protocol=2)
    await websocket.send(pickled_encodings) ##need to change encoding to byte
    eta = await websocket.recv()
    print(eta)



if __name__ == '__main__':
    start_server = websockets.serve(server, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()