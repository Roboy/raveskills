import rospy
import time
from roboy_cognition_msgs.srv import Payment, PaymentResponse


def get_amount_inserted(req):
    time.sleep(2)
    return PaymentResponse(242, "")  # if two scoops are ordered, 42 cents are returned


if __name__ == "__main__":
    rospy.init_node('payment_server')
    rospy.Service('payment', Payment, get_amount_inserted)
    rospy.spin()
