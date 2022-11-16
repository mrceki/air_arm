# !/usr/bin/env python

import rospy
import air_arm_ads.msg
from air_arm_ads.msg import ArmAdsAction
import actionlib

class TrajectoryListener:

    def __init__(self):
        self._action_name = "ArmAdsAction"
        self._feedback = air_arm_ads.msg.ArmAdsFeedback()
        self._result = air_arm_ads.msg.ArmAdsResult()
        self._server = actionlib.SimpleActionServer('ArmAdsAction', ArmAdsAction, execute_cb=self.trajectory_cb, auto_start=False)
        self._server.start()

    def trajectory_cb(self, trajectory):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(req.trajectory))
        # helper variables
        r = rospy.Rate(2)
        success = True

        self._feedback = "START"

        # publish info to the console for the user
        rospy.loginfo('%s: Executing...' % self._action_name)

        for i in range(0, 10):
            self._feedback.state = str(i)
            self._server.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()

        self._result.result = True
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._server.set_succeeded(self._result)


if __name__ == '__main__':
    try:
        rospy.init_node('air_ads_server')
        tl_action_server = TrajectoryListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr(rospy.get_caller_id() + "AIR ADS NODE INITIALIZATION FAILED!!!")
        exit(-1)
