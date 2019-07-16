#! /usr/bin/env python
# import rospy
# import actionlib
# from OrderIceCream.srv import * 	# TODO import from roboy_communication
import time


class ScoopingMockUp(object):

	def __init__(self):
		pass

	def scoop(self, req):
		time.sleep(2)
		return ScoopingResponse(True, "")  # response is success, error_message


if __name__ == "__main__":
	scooping = ScoopingMockUp()
	rospy.init_node('scooping_server')
	rospy.Service('scooping', Scooping, scooping.scoop)
	rospy.spin()


