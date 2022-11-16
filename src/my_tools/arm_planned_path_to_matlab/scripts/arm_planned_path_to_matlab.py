#!/usr/bin/env python
import rospy
from moveit_msgs.msg import DisplayTrajectory
import sys
    
def path_listener():

    rospy.init_node('moveit_planned_path_listener', anonymous=True)

    msg = rospy.wait_for_message("/move_group/display_planned_path", DisplayTrajectory,None)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(msg.trajectory))
    f= open("planned_trajectory.txt","w+")
    f.write(str(msg.trajectory))
    f.close()
    rospy.loginfo("Planned trajectory saved!!!Bye...")
    sys.exit()

if __name__ == '__main__':
    path_listener()