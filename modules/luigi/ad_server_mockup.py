import rospy
import time
from roboy_cognition_msgs.srv import DriveToLocation, DriveToLocationResponse


def get_eta(location):
    time.sleep(2)
    return DriveToLocationResponse(42, "test error msg: {}".format(location))  # 42 seconds until arrival :)


if __name__ == "__main__":
    rospy.init_node('ad_mockup_server')
    rospy.Service('autonomous_driving', DriveToLocation, get_eta)
    rospy.spin()
