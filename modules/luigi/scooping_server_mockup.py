import rospy
import actionlib
from roboy_cognition_msgs.action import OrderIceCreamAction, OrderIceCreamFeedback, OrderIceCreamResult
import time


class ScoopingActionServer:

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "scooping_as", OrderIceCreamAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        success = True
        feedback = OrderIceCreamFeedback()
        rate = rospy.Rate(1)
        finished_flavors = [False for i in range(0, len(goal.flavors))]

        for i in range(0, len(goal.flavors)):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break
            finished_flavors[i] = True
            feedback.finished_flavors = finished_flavors
            self.a_server.publish_feedback(feedback)
            rate.sleep()
            time.sleep(1)   # it takes some time to prepare ice cream

        result = OrderIceCreamResult()
        result.success = True
        result.error_message = ""
        if success:
            self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("scooping_action_server")
    s = ScoopingActionServer()
    rospy.spin()
