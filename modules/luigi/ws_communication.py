import rospy
from roboy_cognition_msgs.srv import DriveToLocation, DriveToLocationResponse
import asyncio
import websockets
import pickle

current_location = ""


def ad_communication(location):
    rospy.wait_for_service('autonomous_driving')
    try:
        drive_to_location = rospy.ServiceProxy('autonomous_driving', DriveToLocation)
        response = drive_to_location(location)
        return response.eta, response.error_message
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)


def communication_with_cloud(server):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    asyncio.get_event_loop().run_until_complete(listen(server))
    print("Received location from Telegram: ", current_location)
    print("Sending location via ROS to AD.")
    eta, err = ad_communication(current_location)
    # print("Received path_found via ROS from AD: ", ) TODO
    print("Received eta via ROS from AD: ", eta)
    print("Sending eta and path_found to Telegram.")
    asyncio.get_event_loop().run_until_complete(say(server, eta))
    print("Finished sending eta and path_found to Telegram.\n")
    loop.close()


async def say(server, eta):
    async with websockets.connect(server+'/pub') as websocket:
        eta_encoding = pickle.dumps(eta)
        await websocket.send(eta_encoding)


async def listen(server):
    global current_location
    async with websockets.connect(server+'/sub') as websocket:
        location_encoding = await websocket.recv()
        current_location = pickle.loads(location_encoding, encoding='bytes')


if __name__ == "__main__":
    ws = 'ws://localhost:8765'  # TODO change to cloud address
    while True:
        communication_with_cloud(ws)
