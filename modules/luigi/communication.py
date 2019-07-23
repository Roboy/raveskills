# import rospy
# from roboy_cognition_msgs.srv import Payment
# from roboy_cognition_msgs.action import OrderIceCreamAction, OrderIceCreamGoal
# import actionlib
import time
import numpy as np


def payment_communication(price, payment_option):
    # rospy.wait_for_service('payment')
    # try:
    #     payment = rospy.ServiceProxy('payment', Payment)
    #     response = payment(np.uint16(price), np.uint8(payment_option))
    #     return response.amount_paid, response.error_message
    # except rospy.ROSInterruptException as e:
    #     print('Service call failed:', e)
    # If luigi module is run without ROS, comment everything from above (including imports) and uncomment this:
    print("in payment communication - price: {} option: {}".format(price, payment_option))
    time.sleep(4)
    return 220, ""


def scooping_feedback_cb(finished_flavors):
    print('Feedback, finished flavors:', finished_flavors)


def scooping_client(flavors, scoops):
    client = actionlib.SimpleActionClient('scooping', OrderIceCreamAction)
    client.wait_for_server()
    goal = OrderIceCreamGoal()
    goal.flavors = flavors
    goal.scoops = scoops
    client.send_goal(goal, feedback_cb=scooping_feedback_cb)
    client.wait_for_result()
    return client.get_result()
    # time.sleep(2)
    # return ScoopingResponse(True, "")  # response is success, error_message


def scooping_communication(flavors, scoops):
    # try:
    #     rospy.init_node('scooping_client')
    #     return scooping_client(flavors, scoops)
    # except rospy.ROSInterruptException as e:
    #     print('Service call failed:', e)
    # If luigi module is run without ROS, comment everything from above (including imports) and uncomment this:
    print("in scooping communication - flavors: {} scoops: {}".format(flavors, scoops))
    time.sleep(4)
    return True, ""
