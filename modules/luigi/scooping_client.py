import rospy
from roboy_cognition_msgs.msg import OrderIceCreamAction, OrderIceCreamGoal
import actionlib
import time
import numpy as np


def scooping_feedback_cb(feedback):
    print('Feedback:', list(feedback.finished_flavors))


def scooping_client(flavors, scoops):
    client = actionlib.SimpleActionClient('scooping_as', OrderIceCreamAction)
    client.wait_for_server()
    goal = OrderIceCreamGoal()
    goal.flavors = flavors
    goal.scoops = scoops
    client.send_goal(goal, feedback_cb=scooping_feedback_cb)
    client.wait_for_result()
    return client.get_result()


# if __name__ == "__main__":
def scooping_client_start(flavors, scoops):
    try:
        rospy.init_node('scooping_client_py')
        # result = scooping_client(["chocolate", "vanilla"], [4, 2])
        # print("Result: ", result)
        rospy.spin()
        result = scooping_client(flavors, scoops)
        return result
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)
    # If luigi module is run without ROS, comment everything from above (including imports) and uncomment this:
    # print("in scooping communication - flavors: {} scoops: {}".format(flavors, scoops))
    # time.sleep(4)
    # return True, ""
