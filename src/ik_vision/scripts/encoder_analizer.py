#!/usr/bin/env python

# subscribe encoder data
# subscribe imu data
# output to csv file


import rospy
from geometry_msgs.msg import Vector3Stamped
import numpy as np

def toDegree(radian):
    return (radian/np.pi)*180.0

def callback(data):
    msg = Vector3Stamped()
    msg.vector.x = data.vector.x
    msg.vector.y = data.vector.y
    msg.vector.z = data.vector.x - data.vector.y
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('encoder_analizer', anonymous=True)

    rospy.Subscriber("/wrist_encoder_rpy", Vector3Stamped, callback)
    pub = rospy.Publisher('wrist_encoder_diff', Vector3Stamped, queue_size=10)

    rospy.spin()
