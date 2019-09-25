import rospy
from std_msgs.msg import Bool


def ad_publisher():
    pub = rospy.Publisher('/roboy/autonomousdriving/arrival', Bool, queue_size=10)
    rospy.init_node('ad_publisher', anonymous=True)
    pub.publish(True)


if __name__ == '__main__':
    try:
        ad_publisher()
    except rospy.ROSInterruptException:
        pass
