# import rospy
# import actionlib
# from roboy_cognition_msgs.srv import OrderIceCream, OrderIceCreamResponse  # TODO adjust when using actionlib?!
import time


def scoop(req):
	time.sleep(2)
	return ScoopingResponse(True, "")  # response is success, error_message


if __name__ == "__main__":
	rospy.init_node('scooping_mockup_server')
	rospy.Service('scooping_mockup', Scooping, scoop)
	rospy.spin()


