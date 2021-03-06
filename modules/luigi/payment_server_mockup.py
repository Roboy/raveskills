import rospy
from roboy_cognition_msgs.srv import Payment, PaymentResponse


def get_amount_inserted(request):
    print("price:", request.price)
    print("payment method:", request.payment_option)
    print("flavors:", request.flavors)
    print("scoops:", request.scoops)
    return PaymentResponse(242, "", "")  # if two scoops are ordered, 42 cents are returned :)


if __name__ == "__main__":
    rospy.init_node('payment_mockup_server')
    rospy.Service('payment', Payment, get_amount_inserted)
    rospy.spin()
