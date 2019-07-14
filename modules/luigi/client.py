import sys
import rospy
from Payment.srv import *

def payment_client(price):
    try:
        rospy.wait_for_service('payment')
        payment = rospy.ServiceProxy('payment', Payment)
        resp = payment(price)
        return resp.state
    except Exception as e:
        print('Service call failed:', e)

if __name__ == "__main__":
    price = int(sys.argv[1])
    print('Requesting', price)
    print('Requested payment:', price, 'received payment:', payment_client(price))