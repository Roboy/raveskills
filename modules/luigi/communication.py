# import rospy
# from roboy_cognition_msgs.srv import Payment
import time
import numpy as np


def payment_communication(price, payment_option):
    # rospy.wait_for_service('payment')
    # try:
    #     payment = rospy.ServiceProxy('payment', Payment)
    #     response = payment(np.uint16(price), np.uint8(payment_option))
    #     return response.amount_paid, response.error_message
    # except Exception as e:
    #     print('Service call failed:', e)
    # If luigi module is run without ROS, comment everything from above (including imports) and uncomment this:
    print("in payment communication - price: {} option: {}".format(price, payment_option))
    time.sleep(4)
    return 220, ""


def scooping_communication(flavors, scoops):
    # TODO add action communication and mock up server for scooping?!
    print("in scooping communication - flavors: {} scoops: {}".format(flavors, scoops))
    time.sleep(4)
    return True, ""
