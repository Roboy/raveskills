import rospy
from roboy_cognition_msgs.srv import *   # import Payment ?


def get_amount_inserted(req):
    return PaymentResponse(self.coin_sum, "")    # TODO add test whether correct values were received, maybe return req?


if __name__ == "__main__":
    rospy.init_node('payment_server')
    rospy.Service('payment', Payment, get_amount_inserted)
    rospy.spin()
