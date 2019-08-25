import rospy
from roboy_cognition_msgs.srv import DriveToLocation, DriveToLocationResponse
import asyncio
import websockets
import pickle

roboy_location = ""
destination_location = ""
connection_type = ""


def ad_communication(location):
    rospy.wait_for_service('autonomous_driving')
    try:
        drive_to_location = rospy.ServiceProxy('autonomous_driving', DriveToLocation)
        response = drive_to_location(location)
        return response.eta, response.path_found, response.error_message
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)


def communication_with_cloud(server):
    global roboy_location
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    asyncio.get_event_loop().run_until_complete(listen(server))
    print("Received location from Telegram: ", destination_location)
    print("Sending location via ROS to AD.")
    if connection_type == "LOCATION_ETA":
        eta, path_found, error_message = ad_communication(destination_location)
        roboy_location = destination_location
        print("Received path_found via ROS from AD: ", path_found)
        print("Received eta via ROS from AD: ", eta)
        print("Sending eta and path_found to Telegram.")
        asyncio.get_event_loop().run_until_complete(say(server, eta, path_found))
        print("Finished sending eta and path_found to Telegram.\n")
    else:
        print("Busy state to Telegram.")
        asyncio.get_event_loop().run_until_complete(say(server=server))
        print("Finished sending busy state to Telegram.\n")
    loop.close()


async def say(server, eta ="", path_found = False):
    async with websockets.connect(server+'/pub') as websocket:
        type_encoding = pickle.dumps(connection_type)
        await websocket.send(type_encoding)
        if connection_type == "BUSY":
            if rospy.has_param("roboy_is_busy"):
                is_busy = rospy.get_param('roboy_is_busy')
            else:
                is_busy = False
            is_busy_encoding = pickle.dumps(is_busy)
            await websocket.send(is_busy_encoding)
            if is_busy:
                roboy_location_encoding = pickle.dumps(roboy_location)
                await websocket.send(roboy_location_encoding)
        elif connection_type == "LOCATION_ETA":
            eta_encoding = pickle.dumps(eta)
            path_found_encoding = pickle.dumps(path_found)
            await websocket.send(eta_encoding)
            await websocket.send(path_found_encoding)


async def listen(server):
    global destination_location
    global connection_type
    async with websockets.connect(server+'/sub') as websocket:
        type_encoding = await websocket.recv()
        connection_type = pickle.loads(type_encoding, encoding='bytes')
        if connection_type == "LOCATION_ETA":
            location_encoding = await websocket.recv()
            destination_location = pickle.loads(location_encoding, encoding='bytes')


if __name__ == "__main__":
    ws = 'ws://localhost:8765'  # TODO change to cloud address
    while True:
        communication_with_cloud(ws)
