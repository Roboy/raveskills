# import rospy
# from Payment.srv import *   # TODO import from roboy_communication
import time


def payment_communication(price, payment_option):
    # rospy.wait_for_service('payment')
    # try:
    #     payment = rospy.ServiceProxy('payment', Payment)
    #     response = payment(price, payment_option)
    #     return response.amount_paid, response.error_message
    # except Exception as e:
    #     print('Service call failed:', e)
    print("in payment communication - price: {} option: {}".format(price, payment_option))
    time.sleep(2)
    return 220, ""


def scooping_communication(flavors, scoops):
    # TODO add action communication and mock up server for scooping?!
    print("in scooping communication - flavors: {} scoops: {}".format(flavors, scoops))
    time.sleep(2)
    return True, ""