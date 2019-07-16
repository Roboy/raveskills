import rospy
from roboy_cognition_msgs.srv import *   # import Payment ?
from enum import Enum


class PaymentOptions(Enum):
    COIN = 0
    PAYPAL = 1


class CoinCounter(object):
    def __init__(self, req):
        self.coin_sum = 1337

    def get_amount_inserted(self):
        return PaymentResponse(self.coin_sum, "")    # TODO add test whether correct values were received...


if __name__ == "__main__":
    coin_counter = CoinCounter()
    rospy.init_node('payment_server')
    rospy.Service('payment', Payment, coin_counter.get_amount_inserted)
    rospy.spin()
