import rospy
import actionlib
from roboy_cognition_msgs.msg import OrderIceCreamAction, OrderIceCreamFeedback, OrderIceCreamResult
import time


class ScoopingActionServer:

    def __init__(self):
        self.a_server = actionlib.SimpleActionServer(
            "scooping_as", OrderIceCreamAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        print("in execute_cb - goal: ", goal)
        success = True
        feedback = OrderIceCreamFeedback()
        rate = rospy.Rate(1)
        finished_scoops = [False for _ in range(0, sum(goal.scoops))]

        print("published feedback")
        for i in range(0, sum(goal.scoops)):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break
            finished_scoops[i] = True
            feedback.finished_scoops = finished_scoops
            feedback.status_message = "more time" if i == 1 else "test status message"
            print("feedback at time {} is {} and {}".format(i, feedback.finished_scoops, feedback.status_message))
            self.a_server.publish_feedback(feedback)
            rate.sleep()
            time.sleep(1)   # it takes some time to prepare ice cream

        result = OrderIceCreamResult()
        result.success = True
        result.error_message = "test"
        if success:
            self.a_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("scooping")
    s = ScoopingActionServer()
    rospy.spin()
