#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys

f_joint = open("air_joint_states_and_raw_efforts.txt", "w+")
is_first = True
ref_stamp = 0.0
raw_efforts = None

def joint_callback(msg):
    global is_first,ref_stamp,raw_efforts
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(msg.header.stamp))
    if(is_first):
        ref_stamp = msg.header.stamp
        is_first = False

    stamp = msg.header.stamp - ref_stamp
#    stamp = msg.header.stamp
    f_joint.write(str(stamp) +", "+str(msg.position)[1:-1] +", "+str(msg.velocity)[1:-1] +", "+str(msg.effort)[1:-1] +", " + str(raw_efforts)[1:-1]+ "\n")
    # for pitch_j, roll_j, yaw_j alignment we indexed raw_efforts  as 1 0 2  

def effort_callback(effort_msg):
    global raw_efforts
    raw_efforts = effort_msg.data

def joint_listener():

    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/air_wrist/joint_states", JointState, joint_callback)
    rospy.Subscriber("/air_wrist/air_wrist_controller/raw_effort", Float64MultiArray, effort_callback)
    rospy.spin()

if __name__ == '__main__':
    joint_listener()
    f_joint.close()
